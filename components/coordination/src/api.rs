use ndarray::Array1;

/// Payload received via the API.
#[derive(Clone, Debug)]
pub struct ApiPayload {
    /// ID of the participant receiving the payload
    pub id: u8,
    /// Position of the particiapant receiving the payload
    pub position: Option<Array1<f64>>,
    /// Radius of the participant receiving the payload
    pub radius: f64,
    /// Inaccuracy in term of the position of the participant receiving the payload
    pub inaccuracy: f64,
    /// Information about other participants in the system
    pub others: Vec<ParticipantInformation>,
}

/// Information about a single other participant.
#[derive(Clone, Debug)]
pub struct ParticipantInformation {
    /// ID of this participant
    pub id: u8,
    /// Current position of this participant
    pub position: Array1<f64>,
    /// Radius of this participant
    pub radius: f64,
    /// Inaccuracy in terms of the position of this participant
    pub inaccuracy: f64,
}
