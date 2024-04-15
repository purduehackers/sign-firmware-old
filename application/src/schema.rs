use bincode::{Decode, Encode};

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
    pub next: u8,
    pub bezier_in: BezierPoint,
    pub bezier_out: BezierPoint,
}

#[derive(Debug, Encode, Decode, Clone, Copy)]
pub struct Key {
    pub time: f32,
    pub red: Option<Color>,
    pub green: Option<Color>,
    pub blue: Option<Color>,
}
