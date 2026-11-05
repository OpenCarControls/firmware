//! The modem's serial link: a buffered reader (PPP needs `BufRead`) and an AT command
//! channel on top of it. Generic over the serial port so it is host-tested with a mock.
//!
//! Every read keeps its state in the struct, not in the future, so a caller can drop a
//! command on a timeout without losing bytes or a half-read line.

use embedded_io_async::{BufRead, ErrorType, Read, ReadReady, Write};
use heapless::String;

use super::at::{self, Final, MAX_LINE_LEN};

/// Adds a read buffer to a serial port, for `BufRead` (what `embassy-net-ppp` reads from).
pub struct BufferedIo<T, const N: usize> {
    inner: T,
    buf: [u8; N],
    start: usize,
    end: usize,
}

impl<T, const N: usize> BufferedIo<T, N> {
    pub fn new(inner: T) -> Self {
        Self {
            inner,
            buf: [0; N],
            start: 0,
            end: 0,
        }
    }
}

impl<T: ErrorType, const N: usize> ErrorType for BufferedIo<T, N> {
    type Error = T::Error;
}

impl<T: Read, const N: usize> BufRead for BufferedIo<T, N> {
    async fn fill_buf(&mut self) -> Result<&[u8], Self::Error> {
        if self.start == self.end {
            self.start = 0;
            self.end = self.inner.read(&mut self.buf).await?;
        }
        Ok(&self.buf[self.start..self.end])
    }

    fn consume(&mut self, amt: usize) {
        self.start = (self.start + amt).min(self.end);
    }
}

impl<T: Read, const N: usize> Read for BufferedIo<T, N> {
    async fn read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
        let data = self.fill_buf().await?;
        let n = data.len().min(out.len());
        out[..n].copy_from_slice(&data[..n]);
        self.consume(n);
        Ok(n)
    }
}

impl<T: Write, const N: usize> Write for BufferedIo<T, N> {
    async fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
        self.inner.write(data).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.inner.flush().await
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelError<E> {
    Io(E),
    /// The port reported end of input.
    Eof,
}

/// Sends AT commands and reads their response lines.
pub struct AtChannel<T, const N: usize> {
    io: BufferedIo<T, N>,
    /// The line being read, kept across dropped reads.
    line: String<MAX_LINE_LEN>,
    /// The line overflowed `MAX_LINE_LEN`: the rest of it is dropped.
    truncated: bool,
}

impl<T, const N: usize> AtChannel<T, N>
where
    T: Read + Write + ReadReady + embedded_io::Read,
{
    pub fn new(port: T) -> Self {
        Self {
            io: BufferedIo::new(port),
            line: String::new(),
            truncated: false,
        }
    }

    #[cfg(test)]
    pub fn port(&self) -> &T {
        &self.io.inner
    }

    /// The buffered port, to run PPP on after `CONNECT`. Bytes the modem sent after
    /// `CONNECT` are still in its buffer, so PPP gets them.
    pub fn data_mode(&mut self) -> &mut BufferedIo<T, N> {
        self.line.clear();
        self.truncated = false;
        &mut self.io
    }

    /// Drops what the modem sent and nobody read: a reply to a command that timed out
    /// mustn't be taken for the next command's.
    pub fn discard_input(&mut self) -> Result<(), ChannelError<T::Error>> {
        self.line.clear();
        self.truncated = false;
        self.io.start = self.io.end;
        let mut scratch = [0u8; 64];
        while self.io.inner.read_ready().map_err(ChannelError::Io)? {
            // Data is ready, so this blocking read returns at once.
            let n = embedded_io::Read::read(&mut self.io.inner, &mut scratch)
                .map_err(ChannelError::Io)?;
            if n == 0 {
                break;
            }
        }
        Ok(())
    }

    /// Writes raw bytes (the `+++` escape) with no line ending.
    pub async fn write_raw(&mut self, data: &[u8]) -> Result<(), ChannelError<T::Error>> {
        self.io.write_all(data).await.map_err(ChannelError::Io)?;
        self.io.flush().await.map_err(ChannelError::Io)
    }

    /// Reads the next non-empty line, without its line ending.
    pub async fn read_line(&mut self) -> Result<String<MAX_LINE_LEN>, ChannelError<T::Error>> {
        loop {
            let data = self.io.fill_buf().await.map_err(ChannelError::Io)?;
            if data.is_empty() {
                return Err(ChannelError::Eof);
            }
            let mut used = 0;
            let mut done = false;
            for &b in data {
                used += 1;
                match b {
                    b'\n' => {
                        done = true;
                        break;
                    }
                    b'\r' => {}
                    _ => {
                        // Anything not printable (line noise at power-on) is dropped.
                        if (0x20..0x7f).contains(&b)
                            && !self.truncated
                            && self.line.push(b as char).is_err()
                        {
                            self.truncated = true;
                        }
                    }
                }
            }
            self.io.consume(used);
            if done {
                let line = core::mem::take(&mut self.line);
                self.truncated = false;
                if !line.trim().is_empty() {
                    return Ok(line);
                }
            }
        }
    }

    /// Sends `cmd` and reads lines until its final result. Every other line (its
    /// information lines and any unsolicited ones) goes to `on_line`; the echo of the
    /// command, if echo is still on, is skipped.
    pub async fn command(
        &mut self,
        cmd: &str,
        mut on_line: impl FnMut(&str),
    ) -> Result<Final, ChannelError<T::Error>> {
        self.discard_input()?;
        self.io
            .write_all(cmd.as_bytes())
            .await
            .map_err(ChannelError::Io)?;
        self.io.write_all(b"\r").await.map_err(ChannelError::Io)?;
        self.io.flush().await.map_err(ChannelError::Io)?;
        loop {
            let line = self.read_line().await?;
            let line = line.trim();
            if line == cmd {
                continue;
            }
            if let Some(result) = at::final_result(line) {
                return Ok(result);
            }
            on_line(line);
        }
    }
}
