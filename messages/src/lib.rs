#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(feature = "std")]
extern crate core;

use core::{marker::PhantomData, ops::Range};

use postcard::{
    accumulator::{CobsAccumulator, FeedResult},
    from_bytes_cobs, to_vec_cobs,
};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub enum Message {
    CMDVel(CMDVelValues),
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct CMDVelValues {
    pub vx: f32,
    pub vy: f32,
    pub vt: f32,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub enum WifiMessage {
    Ping(u64),
    Pong(u64),
}

pub async fn send<T, Tx>(tx: &mut Tx, message: &T) -> Result<(), ()>
where
    T: serde::Serialize,
    Tx: embedded_io_async::Write,
{
    let buf: heapless::Vec<u8, 256> = to_vec_cobs(message).map_err(|_| ())?;

    let zero_index = buf
        .iter()
        .enumerate()
        .find_map(|(i, &b)| if b == 0 { Some(i) } else { None })
        .ok_or(())?;

    let to_transmit = &buf[0..=zero_index];

    tx.write_all(to_transmit).await.map_err(|_| ())
}

#[cfg(feature = "std")]
pub fn send_sync<T>(tx: &mut std::net::UdpSocket, message: &T) -> Result<(), ()>
where
    T: serde::Serialize,
{
    let buf: heapless::Vec<u8, 256> = to_vec_cobs(message).map_err(|_| ())?;

    let zero_index = buf
        .iter()
        .enumerate()
        .find_map(|(i, &b)| if b == 0 { Some(i) } else { None })
        .ok_or(())?;

    let to_transmit = &buf[0..=zero_index];

    tx.send(to_transmit).map(|_| ()).map_err(|_| ())
}

pub struct Receiver<T, const C: usize, const B: usize>
where
    T: for<'de> serde::Deserialize<'de>,
{
    cobs_buf: CobsAccumulator<C>,
    raw_buf: [u8; B],
    window: Range<usize>,
    _unused: PhantomData<T>,
}

impl<T, const C: usize, const B: usize> Receiver<T, C, B>
where
    T: for<'de> serde::Deserialize<'de>,
{
    pub fn new() -> Self {
        Receiver {
            cobs_buf: CobsAccumulator::new(),
            raw_buf: [0u8; B],
            window: 0..0,
            _unused: PhantomData,
        }
    }
}

/// Convert a slice of an array to the corresponding range.
/// If `slice` is not in `parent``, the resulting range will not be valid.
pub fn slice_to_range<T>(parent: &[T], slice: &[T]) -> Range<usize> {
    let start = (slice.as_ptr() as usize - parent.as_ptr() as usize) / core::mem::size_of::<T>();
    let end = start + slice.len();
    start..end
}

impl<T, const C: usize, const B: usize> Receiver<T, C, B>
where
    T: for<'de> serde::Deserialize<'de>,
{
    pub async fn receive<Rx>(&mut self, uart: &mut Rx) -> Result<T, ()>
    where
        Rx: embedded_io_async::Read,
    {
        while !self.window.is_empty() {
            let mut ret: Option<T> = None;

            let window = &self.raw_buf[self.window.clone()];
            self.window = match self.cobs_buf.feed::<T>(&window) {
                FeedResult::Consumed => 0..0,
                FeedResult::OverFull(new_wind) => slice_to_range(&self.raw_buf, new_wind),
                FeedResult::DeserError(new_wind) => slice_to_range(&self.raw_buf, new_wind),
                FeedResult::Success { data, remaining } => {
                    ret = Some(data);
                    slice_to_range(&self.raw_buf, remaining)
                }
            };

            if let Some(ret) = ret {
                return Ok(ret);
            }
        }

        while let Ok(ct) = uart.read(&mut self.raw_buf).await {
            // Finished reading input
            if ct == 0 {
                break;
            }

            self.window = 0..ct;
            while !self.window.is_empty() {
                let mut ret: Option<T> = None;

                let window = &self.raw_buf[self.window.clone()];
                self.window = match self.cobs_buf.feed::<T>(&window) {
                    FeedResult::Consumed => 0..0,
                    FeedResult::OverFull(new_wind) => slice_to_range(&self.raw_buf, new_wind),
                    FeedResult::DeserError(new_wind) => slice_to_range(&self.raw_buf, new_wind),
                    FeedResult::Success { data, remaining } => {
                        ret = Some(data);
                        slice_to_range(&self.raw_buf, remaining)
                    }
                };

                if let Some(ret) = ret {
                    return Ok(ret);
                }
            }
        }

        Err(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test() {
        let serailized: heapless::Vec<u8, 100> = to_vec(&Message::CMDVel(CMDVelValues {
            vx: 0.0,
            vy: 0.0,
            vt: 0.0,
        }))
        .unwrap();
        assert_eq!(
            serailized,
            [
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            ]
        );
    }
}

pub async fn receive_single_read<T, Rx>(rx: &mut Rx) -> Result<T, ()>
where
    T: for<'de> serde::Deserialize<'de>,
    Rx: embedded_io_async::Read,
{
    let mut buf: [u8; 256] = [0; 256];
    let count = rx.read(&mut buf).await.map_err(|_| ())?;

    from_bytes_cobs(&mut buf[0..count]).map_err(|_| ())
}

#[cfg(feature = "std")]
pub fn receive_sync<T>(rx: &mut std::net::UdpSocket) -> Result<T, ()>
where
    T: for<'de> serde::Deserialize<'de>,
{
    let mut buf: [u8; 256] = [0; 256];
    let count = rx.recv(&mut buf).map_err(|_| ())?;

    from_bytes_cobs(&mut buf[0..count]).map_err(|_| ())
}
