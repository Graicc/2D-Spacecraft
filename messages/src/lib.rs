pub enum Message {
    SetRCS(RCSValues);
    Arm(bool);
}

pub struct RCSValues {
    pub fans: [u8; 6];
}
