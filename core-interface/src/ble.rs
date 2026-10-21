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
// Every GATT write/notification is one chunk: a 1-byte header (`LAST` flag in
// bit 7, chunk index in bits 0-6) followed by a slice of the encoded message.
// See `contracts/docs/ble-fragmentation.md`.

use alloc::vec::Vec;

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

/// Largest message that fits in `BLE_MAX_CHUNKS` chunks of `chunk_data` bytes,
/// capped at `BLE_MAX_MESSAGE_BYTES`.
pub fn max_message_len(chunk_data: usize) -> usize {
    BLE_MAX_MESSAGE_BYTES.min(BLE_MAX_CHUNKS * chunk_data)
}

/// Splits `message` into chunks for a link with the given ATT MTU. Each item
/// is `(header, data)`; send them back to back, header first.
pub fn chunks(
    message: &[u8],
    att_mtu: u16,
) -> Result<impl Iterator<Item = (u8, &[u8])>, ChunkError> {
    let chunk_data = chunk_data_size(att_mtu).ok_or(ChunkError::MtuTooSmall)?;
    if message.is_empty() || message.len() > max_message_len(chunk_data) {
        return Err(ChunkError::BadLength);
    }
    let count = message.len().div_ceil(chunk_data);
    Ok(message
        .chunks(chunk_data)
        .enumerate()
        .map(move |(i, data)| {
            let last = if i + 1 == count {
                BLE_CHUNK_LAST_FLAG
            } else {
                0
            };
            (last | i as u8, data)
        }))
}

/// Why a received chunk was dropped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReassemblyError {
    /// Empty chunk, with no header.
    Empty,
    /// A chunk after the first arrived with no partial message.
    NoStart,
    /// The chunk index isn't the one after the previous chunk.
    Gap,
    /// The next chunk came later than `BLE_REASSEMBLY_TIMEOUT_MS`.
    Timeout,
    /// The message would exceed `BLE_MAX_MESSAGE_BYTES`.
    TooLarge,
}

/// Rebuilds messages from chunks. Keeps at most one partial message and drops
/// it on any error; callers feed it `now_ms` (any monotonic clock).
pub struct Reassembler {
    buf: Vec<u8>,
    next_index: Option<u8>,
    last_chunk_ms: u64,
}

impl Reassembler {
    pub const fn new() -> Self {
        Self {
            buf: Vec::new(),
            next_index: None,
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
        let (&header, data) = chunk.split_first().ok_or(ReassemblyError::Empty)?;
        let index = header & BLE_CHUNK_INDEX_MASK;
        let last = header & BLE_CHUNK_LAST_FLAG != 0;

        if index == 0 {
            // A new message always replaces a partial one.
            self.reset();
        } else {
            let Some(expected) = self.next_index else {
                return Err(ReassemblyError::NoStart);
            };
            if now_ms.saturating_sub(self.last_chunk_ms) > BLE_REASSEMBLY_TIMEOUT_MS {
                self.reset();
                return Err(ReassemblyError::Timeout);
            }
            if index != expected {
                self.reset();
                return Err(ReassemblyError::Gap);
            }
        }
        if self.buf.len() + data.len() > BLE_MAX_MESSAGE_BYTES {
            self.reset();
            return Err(ReassemblyError::TooLarge);
        }
        self.buf.extend_from_slice(data);
        self.last_chunk_ms = now_ms;
        if last {
            self.next_index = None;
            return Ok(Some(core::mem::take(&mut self.buf)));
        }
        self.next_index = Some((index + 1) & BLE_CHUNK_INDEX_MASK);
        Ok(None)
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

    fn collect(message: &[u8], mtu: u16) -> Vec<Vec<u8>> {
        chunks(message, mtu)
            .unwrap()
            .map(|(h, d)| {
                let mut c = vec![h];
                c.extend_from_slice(d);
                c
            })
            .collect()
    }

    #[test]
    fn small_message_is_one_chunk_with_last_flag() {
        let out = collect(&[1, 2, 3], BLE_ATT_MTU);
        assert_eq!(out, vec![vec![0x80, 1, 2, 3]]);
    }

    #[test]
    fn chunk_is_never_bigger_than_one_att_packet() {
        let msg = vec![7u8; BLE_MAX_MESSAGE_BYTES];
        let out = collect(&msg, BLE_ATT_MTU);
        assert!(out.iter().all(|c| c.len() <= BLE_MAX_CHUNK_BYTES));
        assert_eq!(out.len(), BLE_MAX_MESSAGE_BYTES.div_ceil(243));
        assert_eq!(out[0][0], 0x00);
        assert_eq!(out[1][0], 0x01);
        assert_eq!(out.last().unwrap()[0], 0x80 | (out.len() as u8 - 1));
    }

    #[test]
    fn message_of_exact_chunk_multiple_ends_on_a_full_chunk() {
        let msg = vec![1u8; 243 * 2];
        let out = collect(&msg, BLE_ATT_MTU);
        assert_eq!(out.len(), 2);
        assert_eq!(out[1][0], 0x81);
        assert_eq!(out[1].len(), 244);
    }

    #[test]
    fn smaller_mtu_gives_smaller_chunks() {
        let msg = vec![1u8; 300];
        let out = collect(&msg, 100);
        assert!(out.iter().all(|c| c.len() <= 97));
        assert_eq!(out.len(), 4);
    }

    #[test]
    fn mtu_above_requested_does_not_grow_chunks() {
        assert_eq!(chunk_data_size(512), Some(243));
    }

    #[test]
    fn mtu_below_minimum_is_refused() {
        assert_eq!(chunk_data_size(BLE_MIN_ATT_MTU - 1), None);
        assert!(matches!(chunks(&[1], 23), Err(ChunkError::MtuTooSmall)));
    }

    #[test]
    fn minimum_mtu_still_carries_the_largest_message() {
        let chunk_data = chunk_data_size(BLE_MIN_ATT_MTU).unwrap();
        assert!(BLE_MAX_CHUNKS * chunk_data >= BLE_MAX_MESSAGE_BYTES);
        assert_eq!(max_message_len(chunk_data), BLE_MAX_MESSAGE_BYTES);
    }

    #[test]
    fn empty_and_oversized_messages_are_refused() {
        assert!(matches!(
            chunks(&[], BLE_ATT_MTU),
            Err(ChunkError::BadLength)
        ));
        let big = vec![0u8; BLE_MAX_MESSAGE_BYTES + 1];
        assert!(matches!(
            chunks(&big, BLE_ATT_MTU),
            Err(ChunkError::BadLength)
        ));
    }

    #[test]
    fn reassembly_round_trips_every_length() {
        for len in [1, 242, 243, 244, 486, 1000, BLE_MAX_MESSAGE_BYTES] {
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
        let msg = vec![5u8; 800];
        let cs = collect(&msg, BLE_ATT_MTU);
        let mut r = Reassembler::new();
        r.push(&cs[0], 0).unwrap();
        assert_eq!(r.push(&cs[2], 1), Err(ReassemblyError::Gap));
        // The rest of the broken message is ignored.
        assert_eq!(r.push(&cs[3], 2), Err(ReassemblyError::NoStart));
        // A new message goes through.
        assert_eq!(r.push(&[0x80, 9], 3).unwrap(), Some(vec![9]));
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
    fn new_start_replaces_a_partial_message() {
        let cs = collect(&vec![5u8; 800], BLE_ATT_MTU);
        let mut r = Reassembler::new();
        r.push(&cs[0], 0).unwrap();
        assert_eq!(r.push(&[0x80, 1, 2], 1).unwrap(), Some(vec![1, 2]));
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
        let data = vec![0u8; 243];
        let mut result = Ok(None);
        for i in 0..BLE_MAX_MESSAGE_BYTES / 243 + 2 {
            let mut c = vec![i as u8];
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
