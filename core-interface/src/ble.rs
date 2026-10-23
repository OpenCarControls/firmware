// BLE transport constants generated from contracts/opencar/core/v1/transport.toml
// and pure utility functions shared by all boards that implement BLE.
include!(concat!(env!("OUT_DIR"), "/ble_transport.rs"));

/// Convert a 6-byte MAC address into a BLE static random address.
///
/// The byte order is reversed (MAC is big-endian; BLE address is little-endian)
/// and the two MSBs of the most-significant octet are forced to `1` as required
/// by the Bluetooth specification for static random addresses.
pub const fn mac_to_ble_address(mac: [u8; 6]) -> [u8; 6] {
    [mac[5], mac[4], mac[3], mac[2], mac[1], mac[0] | 0xC0]
}

// ── Chunking ─────────────────────────────────────────────────────────────────
//
// Every GATT write/notification is one chunk: a 2-byte header (`LAST` flag in
// bit 7 and chunk index in bits 0-6, then the message sequence number)
// followed by a slice of the message stream, which is the encoded message plus
// its big-endian CRC-32. See `contracts/docs/ble-fragmentation.md`.

use alloc::vec::Vec;

/// CRC-32/ISO-HDLC (the zlib CRC-32) of `data`.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

/// Why a message can't be sent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChunkError {
    /// The negotiated ATT MTU is below `BLE_MIN_ATT_MTU`.
    MtuTooSmall,
    /// The message is empty or exceeds the limit for this MTU.
    BadLength,
}

/// Chunk data size for a negotiated ATT MTU: the MTU minus the 3-byte ATT
/// header and the chunk header. `None` when the MTU is below `BLE_MIN_ATT_MTU`.
pub fn chunk_data_size(att_mtu: u16) -> Option<usize> {
    if att_mtu < BLE_MIN_ATT_MTU {
        return None;
    }
    // A bigger MTU than the one the app requests never produces bigger chunks.
    let att_mtu = att_mtu.min(BLE_ATT_MTU) as usize;
    Some(att_mtu - 3 - BLE_CHUNK_HEADER_BYTES)
}

/// Largest message that fits in `BLE_MAX_CHUNKS` chunks of `chunk_data` bytes
/// (with its CRC), capped at `BLE_MAX_MESSAGE_BYTES`.
pub fn max_message_len(chunk_data: usize) -> usize {
    BLE_MAX_MESSAGE_BYTES.min((BLE_MAX_CHUNKS * chunk_data).saturating_sub(BLE_CRC_BYTES))
}

/// Splits `message` into ready-to-send chunks (header included) for a link
/// with the given ATT MTU. `seq` is the message sequence number: the sender
/// increments it (wrapping) per message. Send the chunks back to back.
pub fn chunks(message: &[u8], att_mtu: u16, seq: u8) -> Result<Vec<Vec<u8>>, ChunkError> {
    let chunk_data = chunk_data_size(att_mtu).ok_or(ChunkError::MtuTooSmall)?;
    if message.is_empty() || message.len() > max_message_len(chunk_data) {
        return Err(ChunkError::BadLength);
    }
    let mut stream = Vec::with_capacity(message.len() + BLE_CRC_BYTES);
    stream.extend_from_slice(message);
    stream.extend_from_slice(&crc32(message).to_be_bytes());

    let count = stream.len().div_ceil(chunk_data);
    Ok(stream
        .chunks(chunk_data)
        .enumerate()
        .map(|(i, data)| {
            let last = if i + 1 == count {
                BLE_CHUNK_LAST_FLAG
            } else {
                0
            };
            let mut chunk = Vec::with_capacity(BLE_CHUNK_HEADER_BYTES + data.len());
            chunk.push(last | i as u8);
            chunk.push(seq);
            chunk.extend_from_slice(data);
            chunk
        })
        .collect())
}

/// Why a received chunk was dropped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReassemblyError {
    /// A chunk shorter than its header.
    Empty,
    /// A chunk after the first arrived with no partial message.
    NoStart,
    /// The chunk index isn't the one after the previous chunk.
    Gap,
    /// The chunk belongs to another message than the partial one.
    SeqMismatch,
    /// The next chunk came later than `BLE_REASSEMBLY_TIMEOUT_MS`.
    Timeout,
    /// The message would exceed `BLE_MAX_MESSAGE_BYTES`.
    TooLarge,
    /// The message is shorter than its CRC.
    TooShort,
    /// The CRC doesn't match the message.
    BadCrc,
}

/// Rebuilds messages from chunks. Keeps at most one partial message and drops
/// it on any error; callers feed it `now_ms` (any monotonic clock).
pub struct Reassembler {
    buf: Vec<u8>,
    next_index: Option<u8>,
    seq: u8,
    last_chunk_ms: u64,
}

impl Reassembler {
    pub const fn new() -> Self {
        Self {
            buf: Vec::new(),
            next_index: None,
            seq: 0,
            last_chunk_ms: 0,
        }
    }

    /// Drops any partial message, e.g. on connect or disconnect.
    pub fn reset(&mut self) {
        self.buf.clear();
        self.next_index = None;
    }

    /// Feeds one chunk. Returns the complete message when `chunk` was the last
    /// one. After an error the partial message is gone and chunks are ignored
    /// until the next chunk with index 0.
    pub fn push(&mut self, chunk: &[u8], now_ms: u64) -> Result<Option<Vec<u8>>, ReassemblyError> {
        if chunk.len() < BLE_CHUNK_HEADER_BYTES {
            self.reset();
            return Err(ReassemblyError::Empty);
        }
        let (header, data) = chunk.split_at(BLE_CHUNK_HEADER_BYTES);
        let index = header[0] & BLE_CHUNK_INDEX_MASK;
        let last = header[0] & BLE_CHUNK_LAST_FLAG != 0;
        let seq = header[1];

        if index == 0 {
            // A new message always replaces a partial one.
            self.reset();
            self.seq = seq;
        } else {
            let Some(expected) = self.next_index else {
                return Err(ReassemblyError::NoStart);
            };
            if now_ms.saturating_sub(self.last_chunk_ms) > BLE_REASSEMBLY_TIMEOUT_MS {
                self.reset();
                return Err(ReassemblyError::Timeout);
            }
            if seq != self.seq {
                self.reset();
                return Err(ReassemblyError::SeqMismatch);
            }
            if index != expected {
                self.reset();
                return Err(ReassemblyError::Gap);
            }
        }
        if self.buf.len() + data.len() > BLE_MAX_MESSAGE_BYTES + BLE_CRC_BYTES {
            self.reset();
            return Err(ReassemblyError::TooLarge);
        }
        self.buf.extend_from_slice(data);
        self.last_chunk_ms = now_ms;
        if !last {
            self.next_index = Some((index + 1) & BLE_CHUNK_INDEX_MASK);
            return Ok(None);
        }
        self.next_index = None;
        let mut stream = core::mem::take(&mut self.buf);
        let Some(message_len) = stream.len().checked_sub(BLE_CRC_BYTES) else {
            return Err(ReassemblyError::TooShort);
        };
        let crc = u32::from_be_bytes(stream[message_len..].try_into().unwrap());
        stream.truncate(message_len);
        if crc32(&stream) != crc {
            return Err(ReassemblyError::BadCrc);
        }
        Ok(Some(stream))
    }
}

impl Default for Reassembler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mac_to_ble_address_sets_static_random_bits() {
        let mac = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let addr = mac_to_ble_address(mac);
        assert_eq!(addr[0], 0xFF);
        assert_eq!(addr[1], 0xEE);
        assert_eq!(addr[2], 0xDD);
        assert_eq!(addr[3], 0xCC);
        assert_eq!(addr[4], 0xBB);
        assert_eq!(addr[5], 0xAA | 0xC0);
        assert_eq!(addr[5] & 0xC0, 0xC0, "static-random bits not set");
    }

    #[test]
    fn mac_to_ble_address_all_zeros_still_sets_random_bits() {
        let addr = mac_to_ble_address([0x00; 6]);
        assert_eq!(addr[5] & 0xC0, 0xC0);
        assert_eq!(&addr[0..5], &[0x00; 5]);
    }

    #[test]
    fn gatt_uuids_are_16_bytes_and_distinct() {
        let svc: [u8; 16] = GATT_SERVICE_UUID.to_le_bytes();
        let rx: [u8; 16] = GATT_RX_UUID.to_le_bytes();
        let tx: [u8; 16] = GATT_TX_UUID.to_le_bytes();
        assert_eq!(svc.len(), 16);
        assert_ne!(svc, rx);
        assert_ne!(svc, tx);
        assert_ne!(rx, tx);
    }

    #[test]
    fn gatt_uuids_match_transport_contract() {
        // Values must stay in sync with contracts/opencar/core/v1/transport.toml.
        // If this fails, the transport.toml UUID was changed without a rebuild — or the
        // build.rs codegen is broken.
        assert_eq!(
            GATT_SERVICE_UUID,
            0x1acff001_229b_4d38_a5d2_9af1d9b11f00_u128
        );
        assert_eq!(GATT_RX_UUID, 0x1acff002_229b_4d38_a5d2_9af1d9b11f00_u128);
        assert_eq!(GATT_TX_UUID, 0x1acff003_229b_4d38_a5d2_9af1d9b11f00_u128);
    }

    // ── Chunking ──

    const SEQ: u8 = 7;

    fn collect(message: &[u8], mtu: u16) -> Vec<Vec<u8>> {
        chunks(message, mtu, SEQ).unwrap()
    }

    #[test]
    fn crc32_matches_the_standard_check_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(&[]), 0);
    }

    #[test]
    fn small_message_is_one_chunk_with_last_flag_and_crc() {
        let out = collect(&[0xAA, 0xBB, 0xCC], BLE_ATT_MTU);
        let crc = crc32(&[0xAA, 0xBB, 0xCC]).to_be_bytes();
        let mut expected = vec![0x80, SEQ, 0xAA, 0xBB, 0xCC];
        expected.extend_from_slice(&crc);
        assert_eq!(out, vec![expected]);
    }

    #[test]
    fn headers_count_up_and_the_last_chunk_carries_the_flag() {
        // 500 bytes + 4 CRC = 504 = 242 + 242 + 20 at the requested MTU.
        let out = collect(&vec![1u8; 500], BLE_ATT_MTU);
        assert_eq!(
            out.iter().map(|c| c[0]).collect::<Vec<_>>(),
            [0x00, 0x01, 0x82]
        );
        assert!(out.iter().all(|c| c[1] == SEQ));
        assert_eq!(
            out.iter().map(|c| c.len()).collect::<Vec<_>>(),
            [244, 244, 22]
        );
    }

    #[test]
    fn chunk_is_never_bigger_than_one_att_packet() {
        let msg = vec![7u8; BLE_MAX_MESSAGE_BYTES];
        let out = collect(&msg, BLE_ATT_MTU);
        assert!(out.iter().all(|c| c.len() <= BLE_MAX_CHUNK_BYTES));
        assert_eq!(
            out.len(),
            (BLE_MAX_MESSAGE_BYTES + BLE_CRC_BYTES).div_ceil(242)
        );
    }

    #[test]
    fn crc_can_straddle_the_last_two_chunks() {
        // 240 + 4 CRC = 244 bytes: 242 in the first chunk, 2 CRC bytes after.
        let msg = vec![3u8; 240];
        let out = collect(&msg, BLE_ATT_MTU);
        assert_eq!(out.len(), 2);
        assert_eq!(out[1].len(), BLE_CHUNK_HEADER_BYTES + 2);
        let mut r = Reassembler::new();
        assert_eq!(r.push(&out[0], 0).unwrap(), None);
        assert_eq!(r.push(&out[1], 0).unwrap(), Some(msg));
    }

    #[test]
    fn smaller_mtu_gives_smaller_chunks() {
        let out = collect(&vec![1u8; 300], 100);
        assert!(out.iter().all(|c| c.len() <= 97));
        assert_eq!(out.len(), 4);
    }

    #[test]
    fn mtu_above_requested_does_not_grow_chunks() {
        assert_eq!(chunk_data_size(512), Some(242));
    }

    #[test]
    fn mtu_below_minimum_is_refused() {
        assert_eq!(chunk_data_size(BLE_MIN_ATT_MTU - 1), None);
        assert!(matches!(chunks(&[1], 23, 0), Err(ChunkError::MtuTooSmall)));
    }

    #[test]
    fn minimum_mtu_still_carries_the_largest_message() {
        let chunk_data = chunk_data_size(BLE_MIN_ATT_MTU).unwrap();
        assert!(BLE_MAX_CHUNKS * chunk_data >= BLE_MAX_MESSAGE_BYTES + BLE_CRC_BYTES);
        assert_eq!(max_message_len(chunk_data), BLE_MAX_MESSAGE_BYTES);
        let out = collect(&vec![1u8; BLE_MAX_MESSAGE_BYTES], BLE_MIN_ATT_MTU);
        assert!(out.len() <= BLE_MAX_CHUNKS);
    }

    #[test]
    fn empty_and_oversized_messages_are_refused() {
        assert!(matches!(
            chunks(&[], BLE_ATT_MTU, 0),
            Err(ChunkError::BadLength)
        ));
        let big = vec![0u8; BLE_MAX_MESSAGE_BYTES + 1];
        assert!(matches!(
            chunks(&big, BLE_ATT_MTU, 0),
            Err(ChunkError::BadLength)
        ));
    }

    #[test]
    fn reassembly_round_trips_every_length() {
        for len in [
            1,
            238,
            239,
            240,
            241,
            242,
            243,
            484,
            1000,
            BLE_MAX_MESSAGE_BYTES,
        ] {
            let msg: Vec<u8> = (0..len).map(|i| i as u8).collect();
            let mut r = Reassembler::new();
            let mut done = None;
            for c in collect(&msg, BLE_ATT_MTU) {
                done = r.push(&c, 0).unwrap();
            }
            assert_eq!(done.as_deref(), Some(msg.as_slice()), "len {len}");
        }
    }

    #[test]
    fn reassembler_drops_a_gap_and_waits_for_the_next_start() {
        let cs = collect(&vec![5u8; 800], BLE_ATT_MTU);
        let mut r = Reassembler::new();
        r.push(&cs[0], 0).unwrap();
        assert_eq!(r.push(&cs[2], 1), Err(ReassemblyError::Gap));
        // The rest of the broken message is ignored.
        assert_eq!(r.push(&cs[3], 2), Err(ReassemblyError::NoStart));
        // A new message goes through.
        let next = collect(&[9], BLE_ATT_MTU);
        assert_eq!(r.push(&next[0], 3).unwrap(), Some(vec![9]));
    }

    #[test]
    fn reassembler_drops_a_duplicate_chunk() {
        let cs = collect(&vec![5u8; 800], BLE_ATT_MTU);
        let mut r = Reassembler::new();
        r.push(&cs[0], 0).unwrap();
        r.push(&cs[1], 0).unwrap();
        assert_eq!(r.push(&cs[1], 0), Err(ReassemblyError::Gap));
    }

    #[test]
    fn chunks_of_two_messages_are_never_spliced() {
        // A's chunk 1 and B's chunk 0 are lost; B's chunk 1 has the index the
        // receiver is waiting for, so only the sequence number catches it.
        let a = chunks(&vec![1u8; 800], BLE_ATT_MTU, 1).unwrap();
        let b = chunks(&vec![2u8; 800], BLE_ATT_MTU, 2).unwrap();
        let mut r = Reassembler::new();
        r.push(&a[0], 0).unwrap();
        assert_eq!(r.push(&b[1], 1), Err(ReassemblyError::SeqMismatch));
    }

    #[test]
    fn a_corrupted_message_fails_the_crc() {
        let mut cs = collect(&vec![5u8; 800], BLE_ATT_MTU);
        cs[1][5] ^= 0x01;
        let mut r = Reassembler::new();
        let mut result = Ok(None);
        for c in &cs {
            result = r.push(c, 0);
        }
        assert_eq!(result, Err(ReassemblyError::BadCrc));
    }

    #[test]
    fn a_message_shorter_than_its_crc_is_dropped() {
        let mut r = Reassembler::new();
        assert_eq!(r.push(&[0x80, 0, 1, 2], 0), Err(ReassemblyError::TooShort));
    }

    #[test]
    fn new_start_replaces_a_partial_message() {
        let cs = collect(&vec![5u8; 800], BLE_ATT_MTU);
        let mut r = Reassembler::new();
        r.push(&cs[0], 0).unwrap();
        let next = chunks(&[1, 2], BLE_ATT_MTU, 9).unwrap();
        assert_eq!(r.push(&next[0], 1).unwrap(), Some(vec![1, 2]));
    }

    #[test]
    fn reassembler_times_out_between_chunks() {
        let cs = collect(&vec![5u8; 800], BLE_ATT_MTU);
        let mut r = Reassembler::new();
        r.push(&cs[0], 1000).unwrap();
        assert!(r.push(&cs[1], 1000 + BLE_REASSEMBLY_TIMEOUT_MS).is_ok());
        assert_eq!(
            r.push(&cs[2], 1000 + 2 * BLE_REASSEMBLY_TIMEOUT_MS + 1),
            Err(ReassemblyError::Timeout)
        );
    }

    #[test]
    fn reassembler_rejects_oversize_messages() {
        let mut r = Reassembler::new();
        let data = vec![0u8; 242];
        let mut result = Ok(None);
        for i in 0..(BLE_MAX_MESSAGE_BYTES + BLE_CRC_BYTES) / 242 + 2 {
            let mut c = vec![i as u8, 0];
            c.extend_from_slice(&data);
            result = r.push(&c, 0);
            if result.is_err() {
                break;
            }
        }
        assert_eq!(result, Err(ReassemblyError::TooLarge));
    }

    #[test]
    fn reset_drops_the_partial_message() {
        let cs = collect(&vec![5u8; 800], BLE_ATT_MTU);
        let mut r = Reassembler::new();
        r.push(&cs[0], 0).unwrap();
        r.reset();
        assert_eq!(r.push(&cs[1], 0), Err(ReassemblyError::NoStart));
        assert_eq!(r.push(&[], 0), Err(ReassemblyError::Empty));
    }
}
