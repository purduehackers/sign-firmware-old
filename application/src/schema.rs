use bincode::{Encode, Decode};

#[derive(Debug, Encode, Decode, Clone, Copy)]
pub struct Header {
    pub duration: f32,
    pub frames: [u16; 5],
}

#[derive(Debug, Encode, Decode, Clone, Copy)]
pub struct BezierPoint {
    pub x: f32,
    pub y: f32,
}

#[derive(Debug, Encode, Decode, Clone, Copy)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

#[derive(Debug, Encode, Decode, Clone, Copy)]
pub struct Key {
    pub time: f32,
    pub bezier_in: BezierPoint,
    pub bezier_out: BezierPoint,
    pub color: Color,
}
