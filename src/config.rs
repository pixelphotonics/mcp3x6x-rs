
trait BitField: Sized {
    const MASK: u8;
    const SHIFT: u8;

    fn as_u8_unshifted(self) -> u8;

    fn set(self, target: &mut u8) {
        *target = (*target & !Self::MASK) | (self.as_u8_unshifted() << Self::SHIFT);
    }

    fn get(value: u8) -> Self;
}

pub struct BoolBitfield<const SHIFT: u8>(bool);

impl<const SHIFT: u8> BitField for BoolBitfield<SHIFT> {
    const MASK: u8 = 0b1 << SHIFT;
    const SHIFT: u8 = SHIFT;

    fn as_u8_unshifted(self) -> u8 {
        if self.0 { Self::MASK >> Self::SHIFT } else { 0 }
    }

    fn get(value: u8) -> Self {
        BoolBitfield(value & Self::MASK != 0)
    }
}

impl<const SHIFT: u8> BoolBitfield<SHIFT> {
    fn new(bit: bool) -> Self {
        BoolBitfield(bit)
    }
}


pub trait Register {
    const WRTIABLE: bool = true;

    fn address() -> u8;
    fn size() -> usize;

    fn to_bytes(&self, out: &mut [u8]);
    fn try_from_bytes(bytes: &[u8]) -> Option<Self> where Self: Sized;
}


#[repr(u8)]
pub enum AdcMode {
    Conversion = 0b11,
    Standby = 0b10,
    Shutdown = 0b00,
}

impl BitField for AdcMode {
    const MASK: u8 = 0b0000_0011;
    const SHIFT: u8 = 0;

    fn as_u8_unshifted(self) -> u8 {
        self as u8
    }
    
    fn get(value: u8) -> Self {
        match (value & Self::MASK) >> Self::SHIFT {
            0b11 => AdcMode::Conversion,
            0b10 => AdcMode::Standby,
            0b01 => AdcMode::Shutdown,
            0b00 => AdcMode::Shutdown,
            _ => panic!("Invalid ADC mode"),
        }
    }
}

#[repr(u8)]
pub enum CurrentSource {
    NoSource = 0b00,
    Source0_9uA = 0b01,
    Source3_7uA = 0b10,
    Source15uA = 0b11,
}

impl BitField for CurrentSource {
    const MASK: u8 = 0b0000_1100;
    const SHIFT: u8 = 2;

    fn as_u8_unshifted(self) -> u8 {
        self as u8
    }
    
    fn get(value: u8) -> Self {
        match (value & Self::MASK) >> Self::SHIFT {
            0b00 => CurrentSource::NoSource,
            0b01 => CurrentSource::Source0_9uA,
            0b10 => CurrentSource::Source3_7uA,
            0b11 => CurrentSource::Source15uA,
            _ => panic!("Invalid current source"),
        }
    }
}


#[repr(u8)]
pub enum ClockSelection {
    Internal = 0b11,
    InternalNoOutput = 0b10,
    External = 0b00,
}

impl BitField for ClockSelection {
    const MASK: u8 = 0b0011_0000;
    const SHIFT: u8 = 4;

    fn as_u8_unshifted(self) -> u8 {
        self as u8
    }
    
    fn get(value: u8) -> Self {
        match (value & Self::MASK) >> Self::SHIFT {
            0b11 => ClockSelection::Internal,
            0b10 => ClockSelection::InternalNoOutput,
            0b01 => ClockSelection::External,
            0b00 => ClockSelection::External,
            _ => unreachable!("Invalid clock selection"),
        }
    }
}


#[repr(u8)]
pub enum VoltageReference {
    Internal = 0b1,
    External = 0b0,
}

impl BitField for VoltageReference {
    const MASK: u8 = 0b1000_0000;
    const SHIFT: u8 = 7;

    fn as_u8_unshifted(self) -> u8 {
        self as u8
    }
    
    fn get(value: u8) -> Self {
        match (value & Self::MASK) >> Self::SHIFT {
            0b1 => VoltageReference::Internal,
            0b0 => VoltageReference::External,
            _ => unreachable!("Invalid voltage reference"),
        }
    }
}


#[derive(Debug, Clone, Copy)]
pub struct RegisterConfig0(u8);

impl Default for RegisterConfig0 {
    fn default() -> Self {
        RegisterConfig0(0b1100_0000)
    }
}

impl Register for RegisterConfig0 {
    fn address() -> u8 {
        0x01
    }

    fn size() -> usize {
        1
    }

    fn to_bytes(&self, out: &mut [u8]) {
        out[0] = self.0;
    }
    
    fn try_from_bytes(bytes: &[u8]) -> Option<Self> {
        Some(Self(bytes[0]))
    }
}

impl RegisterConfig0 {
    pub fn voltage_reference(&self) -> VoltageReference {
        VoltageReference::get(self.0)
    }

    pub fn set_voltage_reference(&mut self, voltage_reference: VoltageReference) {
        voltage_reference.set(&mut self.0);
    }

    pub fn clock_selection(&self) -> ClockSelection {
        ClockSelection::get(self.0)
    }

    pub fn set_clock_selection(&mut self, clock_selection: ClockSelection) {
        clock_selection.set(&mut self.0);
    }

    pub fn current_source(&self) -> CurrentSource {
        CurrentSource::get(self.0)
    }

    pub fn set_current_source(&mut self, current_source: CurrentSource) {
        current_source.set(&mut self.0);
    }

    pub fn adc_mode(&self) -> AdcMode {
        AdcMode::get(self.0)
    }

    pub fn set_adc_mode(mut self, adc_mode: AdcMode) -> Self {
        adc_mode.set(&mut self.0);
        self
    }
}











/// Prescaler Value Selection for AMCLK
#[repr(u8)]
pub enum PrescalerValue {
    Div1 = 0b00,
    Div2 = 0b01,
    Div4 = 0b10,
    Div8 = 0b11,
}

impl BitField for PrescalerValue {
    const MASK: u8 = 0b1100_0000;
    const SHIFT: u8 = 6;
    
    fn as_u8_unshifted(self) -> u8 {
        self as u8
    }
    
    fn get(value: u8) -> Self {
        match (value & Self::MASK) >> Self::SHIFT {
            0b00 => PrescalerValue::Div1,
            0b01 => PrescalerValue::Div2,
            0b10 => PrescalerValue::Div4,
            0b11 => PrescalerValue::Div8,
            _ => unreachable!("Invalid prescaler value"),
        }
    }
}

/// Oversampling Ratio for Delta-Sigma A/D Conversion
#[repr(u8)]
pub enum OversamplingRatio {
    Ratio32 = 0b0000,
    Ratio64 = 0b0001,
    Ratio128 = 0b0010,
    Ratio256 = 0b0011,
    Ratio512 = 0b0100,
    Ratio1024 = 0b0101,
    Ratio2048 = 0b0110,
    Ratio4096 = 0b0111,
    Ratio8192 = 0b1000,
    Ratio16384 = 0b1001,
    Ratio20480 = 0b1010,
    Ratio24576 = 0b1011,
    Ratio40960 = 0b1100,
    Ratio49152 = 0b1101,
    Ratio81920 = 0b1110,
    Ratio98304 = 0b1111,
}

impl BitField for OversamplingRatio {
    const MASK: u8 = 0b0011_1100;
    const SHIFT: u8 = 2;

    fn as_u8_unshifted(self) -> u8 {
        self as u8
    }
    
    fn get(value: u8) -> Self {
        match (value & Self::MASK) >> Self::SHIFT {
            0b0000 => OversamplingRatio::Ratio32,
            0b0001 => OversamplingRatio::Ratio64,
            0b0010 => OversamplingRatio::Ratio128,
            0b0011 => OversamplingRatio::Ratio256,
            0b0100 => OversamplingRatio::Ratio512,
            0b0101 => OversamplingRatio::Ratio1024,
            0b0110 => OversamplingRatio::Ratio2048,
            0b0111 => OversamplingRatio::Ratio4096,
            0b1000 => OversamplingRatio::Ratio8192,
            0b1001 => OversamplingRatio::Ratio16384,
            0b1010 => OversamplingRatio::Ratio20480,
            0b1011 => OversamplingRatio::Ratio24576,
            0b1100 => OversamplingRatio::Ratio40960,
            0b1101 => OversamplingRatio::Ratio49152,
            0b1110 => OversamplingRatio::Ratio81920,
            0b1111 => OversamplingRatio::Ratio98304,
            _ => unreachable!("Invalid oversampling ratio"),
        }
    }
}


#[derive(Debug, Clone, Copy)]
pub struct RegisterConfig1(u8);

impl Register for RegisterConfig1 {
    fn address() -> u8 {
        0x02
    }

    fn size() -> usize {
        1
    }

    fn to_bytes(&self, out: &mut [u8]) {
        out[0] = self.0;
    }
    
    fn try_from_bytes(bytes: &[u8]) -> Option<Self> {
        Some(Self(bytes[0]))
    }
}

impl Default for RegisterConfig1 {
    fn default() -> Self {
        Self(0b0000_1100)
    }
}

impl RegisterConfig1 {
    pub fn prescaler_value(&self) -> PrescalerValue {
        PrescalerValue::get(self.0)
    }

    pub fn set_prescaler_value(&mut self, prescaler_value: PrescalerValue) {
        prescaler_value.set(&mut self.0);
    }

    pub fn oversampling_ratio(&self) -> OversamplingRatio {
        OversamplingRatio::get(self.0)
    }

    pub fn set_oversampling_ratio(&mut self, oversampling_ratio: OversamplingRatio) {
        oversampling_ratio.set(&mut self.0);
    }
}






/// ADC Bias current selection
#[repr(u8)]
pub enum BoostMode {
    Boost2 = 0b11,
    Boost1 = 0b10,
    Boost0_66 = 0b01,
    Boost0_5 = 0b00,
}

impl BitField for BoostMode {
    const MASK: u8 = 0b1100_0000;
    const SHIFT: u8 = 6;

    fn as_u8_unshifted(self) -> u8 {
        self as u8
    }
    
    fn get(value: u8) -> Self {
        match (value & Self::MASK) >> Self::SHIFT {
            0b11 => BoostMode::Boost2,
            0b10 => BoostMode::Boost1,
            0b01 => BoostMode::Boost0_66,
            0b00 => BoostMode::Boost0_5,
            _ => unreachable!("Invalid boost mode"),
        }
    }
}


/// ADC Gain selection
#[repr(u8)]
pub enum Gain {
    Gain0_33 = 0b000,
    Gain1 = 0b001,
    Gain2 = 0b010,
    Gain4 = 0b011,
    Gain8 = 0b100,
    Gain16 = 0b101,
    Gain32 = 0b110,
    Gain64 = 0b111,
}

impl BitField for Gain {
    const MASK: u8 = 0b0011_1000;
    const SHIFT: u8 = 3;

    fn as_u8_unshifted(self) -> u8 {
        self as u8
    }
    
    fn get(value: u8) -> Self {
        match (value & Self::MASK) >> Self::SHIFT {
            0b000 => Gain::Gain0_33,
            0b001 => Gain::Gain1,
            0b010 => Gain::Gain2,
            0b011 => Gain::Gain4,
            0b100 => Gain::Gain8,
            0b101 => Gain::Gain16,
            0b110 => Gain::Gain32,
            0b111 => Gain::Gain64,
            _ => unreachable!("Invalid gain"),
        }
    }

}

#[repr(u8)]
pub enum AutoZeroing {
    Enabled = 0b1,
    Disabled = 0b0,
}

impl BitField for AutoZeroing {
    const MASK: u8 = 0b0000_0100;
    const SHIFT: u8 = 2;

    fn as_u8_unshifted(self) -> u8 {
        self as u8
    }
    
    fn get(value: u8) -> Self {
        match (value & Self::MASK) >> Self::SHIFT {
            0b1 => AutoZeroing::Enabled,
            0b0 => AutoZeroing::Disabled,
            _ => unreachable!("Invalid auto zeroing"),
        }
    }
}


#[repr(u8)]
pub enum AutoZeroReference {
    /// Internal voltage reference buffer chopping algorithm is enabled. This setting has no effect when external voltage reference is selected.
    Internal = 0b1,

    /// External voltage reference buffer chopping algorithm is enabled.
    External = 0b0,
}

impl BitField for AutoZeroReference {
    const MASK: u8 = 0b0000_0010;
    const SHIFT: u8 = 1;

    fn as_u8_unshifted(self) -> u8 {
        self as u8
    }
    
    fn get(value: u8) -> Self {
        match (value & Self::MASK) >> Self::SHIFT {
            0b1 => AutoZeroReference::Internal,
            0b0 => AutoZeroReference::External,
            _ => unreachable!("Invalid auto zero reference"),
        }
    }

}




#[derive(Debug, Clone, Copy)]
pub struct RegisterConfig2(u8);

impl Register for RegisterConfig2 {
    fn address() -> u8 {
        0x03
    }

    fn size() -> usize {
        1
    }

    fn to_bytes(&self, out: &mut [u8]) {
        out[0] = self.0;
    }
    
    fn try_from_bytes(bytes: &[u8]) -> Option<Self> {
        Some(Self(bytes[0]))
    }
}

impl Default for RegisterConfig2 {
    fn default() -> Self {
        Self(0b10001011)
    }
}

impl RegisterConfig2 {
    pub fn boost_mode(&self) -> BoostMode {
        BoostMode::get(self.0)
    }

    pub fn set_boost_mode(&mut self, boost_mode: BoostMode) {
        boost_mode.set(&mut self.0);
    }

    pub fn gain(&self) -> Gain {
        Gain::get(self.0)
    }

    pub fn set_gain(&mut self, gain: Gain) {
        gain.set(&mut self.0);
    }

    pub fn auto_zeroing(&self) -> AutoZeroing {
        AutoZeroing::get(self.0)
    }

    pub fn set_auto_zeroing(&mut self, auto_zeroing: AutoZeroing) {
        auto_zeroing.set(&mut self.0);
    }

    pub fn auto_zero_reference(&self) -> AutoZeroReference {
        AutoZeroReference::get(self.0)
    }

    pub fn set_auto_zero_reference(&mut self, auto_zero_reference: AutoZeroReference) {
        auto_zero_reference.set(&mut self.0);
    }
}












pub enum ConversionMode {
    Continuous = 0b11,
    SingleStandby = 0b10,
    SingleShutdown = 0b00,
}

impl BitField for ConversionMode {
    const MASK: u8 = 0b1100_0000;
    const SHIFT: u8 = 6;

    fn as_u8_unshifted(self) -> u8 {
        self as u8
    }
    
    fn get(value: u8) -> Self {
        match (value & Self::MASK) >> Self::SHIFT {
            0b11 => ConversionMode::Continuous,
            0b10 => ConversionMode::SingleStandby,
            0b01 | 0b00 => ConversionMode::SingleShutdown,
            _ => unreachable!("Invalid conversion mode"),
        }
    }

}


pub enum DataFormat {
    /// 32-bit (25-bit data with 4-bit sign extension and 4 bit Channel ID)
    Size32Data25WithId = 0b11,

    /// 32-bit (25-bit data with 8 bit sign extension)
    Size32Data25 = 0b10,

    /// 32-bit (24-bit left justified data)
    Size32Data24Left = 0b01,

    /// 24-bit size with 24-bit data
    Size24 = 0b00,
}

impl BitField for DataFormat {
    const MASK: u8 = 0b0011_0000;
    const SHIFT: u8 = 4;

    fn as_u8_unshifted(self) -> u8 {
        self as u8
    }
    
    fn get(value: u8) -> Self {
        match (value & Self::MASK) >> Self::SHIFT {
            0b11 => DataFormat::Size32Data25WithId,
            0b10 => DataFormat::Size32Data25,
            0b01 => DataFormat::Size32Data24Left,
            0b00 => DataFormat::Size24,
            _ => unreachable!("Invalid data format"),
        }
    }

}

pub enum CRCRead {
    CRC32 = 0b11,
    CRC16 = 0b01,
    Disabled = 0b00,
}

impl BitField for CRCRead {
    const MASK: u8 = 0b0000_1100;
    const SHIFT: u8 = 2;

    fn as_u8_unshifted(self) -> u8 {
        self as u8
    }
    
    fn get(value: u8) -> Self {
        match (value & Self::MASK) >> Self::SHIFT {
            0b11 => CRCRead::CRC32,
            0b01 => CRCRead::CRC16,
            0b00 | 0b10 => CRCRead::Disabled,
            _ => unreachable!("Invalid CRC read"),
        }
    }
}

pub enum DigitalOffsetCalibration {
    Enabled = 0b1,
    Disabled = 0b0,
}

impl BitField for DigitalOffsetCalibration {
    const MASK: u8 = 0b0000_0010;
    const SHIFT: u8 = 1;

    fn as_u8_unshifted(self) -> u8 {
        self as u8
    }
    
    fn get(value: u8) -> Self {
        match (value & Self::MASK) >> Self::SHIFT {
            0b1 => DigitalOffsetCalibration::Enabled,
            0b0 => DigitalOffsetCalibration::Disabled,
            _ => unreachable!("Invalid digital offset calibration"),
        }
    }
}

pub enum DigitalGainCalibration {
    Enabled = 0b1,
    Disabled = 0b0,
}

impl BitField for DigitalGainCalibration {
    const MASK: u8 = 0b0000_0001;
    const SHIFT: u8 = 0;

    fn as_u8_unshifted(self) -> u8 {
        self as u8
    }
    
    fn get(value: u8) -> Self {
        match (value & Self::MASK) >> Self::SHIFT {
            0b1 => DigitalGainCalibration::Enabled,
            0b0 => DigitalGainCalibration::Disabled,
            _ => unreachable!("Invalid digital gain calibration"),
        }
    }
}


#[derive(Debug, Clone, Copy)]
pub struct RegisterConfig3(u8);

impl Register for RegisterConfig3 {
    fn address() -> u8 {
        0x04
    }

    fn size() -> usize {
        1
    }

    fn to_bytes(&self, out: &mut [u8]) {
        out[0] = self.0;
    }
    
    fn try_from_bytes(bytes: &[u8]) -> Option<Self> {
        Some(Self(bytes[0]))
    }
}

impl Default for RegisterConfig3 {
    fn default() -> Self {
        Self(0b0000_0000)
    }
}

impl RegisterConfig3 {
    pub fn conversion_mode(&self) -> ConversionMode {
        ConversionMode::get(self.0)
    }

    pub fn set_conversion_mode(&mut self, conversion_mode: ConversionMode) {
        conversion_mode.set(&mut self.0);
    }

    pub fn data_format(&self) -> DataFormat {
        DataFormat::get(self.0)
    }

    pub fn set_data_format(&mut self, data_format: DataFormat) {
        data_format.set(&mut self.0);
    }

    pub fn crc_read(&self) -> CRCRead {
        CRCRead::get(self.0)
    }

    pub fn set_crc_read(&mut self, crc_read: CRCRead) {
        crc_read.set(&mut self.0);
    }

    pub fn digital_offset_calibration(&self) -> DigitalOffsetCalibration {
        DigitalOffsetCalibration::get(self.0)
    }

    pub fn set_digital_offset_calibration(&mut self, digital_offset_calibration: DigitalOffsetCalibration) {
        digital_offset_calibration.set(&mut self.0);
    }

    pub fn digital_gain_calibration(&self) -> DigitalGainCalibration {
        DigitalGainCalibration::get(self.0)
    }

    pub fn set_digital_gain_calibration(&mut self, digital_gain_calibration: DigitalGainCalibration) {
        digital_gain_calibration.set(&mut self.0);
    }
}



pub type DRStatus = BoolBitfield<6>;
type CRCCFGStatus = BoolBitfield<5>;
type PORStatus = BoolBitfield<4>;
type EnFastCmd = BoolBitfield<1>;
type EnSTP = BoolBitfield<0>;

pub enum IRQMode {
    MDATOut_IRQ_High = 0b11,
    MDATOut_IRQ_Z = 0b10,

    IRQ_Out_IRQ_High = 0b01,
    IRQ_Out_IRQ_Z = 0b00,
}

impl BitField for IRQMode {
    const MASK: u8 = 0b0000_1100;
    const SHIFT: u8 = 6;

    fn as_u8_unshifted(self) -> u8 {
        self as u8
    }

    fn get(value: u8) -> Self {
        match (value & Self::MASK) >> Self::SHIFT {
            0b11 => IRQMode::MDATOut_IRQ_High,
            0b10 => IRQMode::MDATOut_IRQ_Z,
            0b01 => IRQMode::IRQ_Out_IRQ_High,
            0b00 => IRQMode::IRQ_Out_IRQ_Z,
            _ => unreachable!("Invalid IRQ mode"),
        }
    }
}




#[derive(Debug, Clone, Copy)]
pub struct RegisterIRQ(u8);

impl Register for RegisterIRQ {
    fn address() -> u8 {
        0x05
    }

    fn size() -> usize {
        1
    }

    fn to_bytes(&self, out: &mut [u8]) {
        out[0] = self.0;
    }
    
    fn try_from_bytes(bytes: &[u8]) -> Option<Self> {
        Some(Self(bytes[0]))
    }
}

impl Default for RegisterIRQ {
    fn default() -> Self {
        Self(0b01110011)
    }
}

impl RegisterIRQ {
    pub fn dr_status(&self) -> DRStatus {
        DRStatus::get(self.0)
    }

    pub fn crccfg_status(&self) -> CRCCFGStatus {
        CRCCFGStatus::get(self.0)
    }

    pub fn por_status(&self) -> PORStatus {
        PORStatus::get(self.0)
    }

    pub fn fast_commands_enabled(&self) -> bool {
        EnFastCmd::get(self.0).0
    }

    pub fn set_fast_commands_enabled(&mut self, enable: bool){
        EnFastCmd::new(enable).set(&mut self.0);
    }

    pub fn start_interrupt_output_enabled(&self) -> bool {
        EnSTP::get(self.0).0
    }

    pub fn set_start_interrupt_output_enabled(&mut self, enable: bool) {
        EnSTP::new(enable).set(&mut self.0);
    }

    pub fn irq_mode(&self) -> IRQMode {
        IRQMode::get(self.0)
    }

    pub fn set_irq_mode(&mut self, irq_mode: IRQMode) {
        irq_mode.set(&mut self.0);
    }
}





#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum MuxSelection {
    InternalVcm = 0b1111,
    InternalTempSensorM = 0b1110,
    InternalTempSensorP = 0b1101,
    RefinM = 0b1100,
    RefinP = 0b1011,
    _Reserved = 0b1010,
    AnalogVdd = 0b1001,
    AnalogGnd = 0b1000,
    Channel7 = 0b0111,
    Channel6 = 0b0110,
    Channel5 = 0b0101,
    Channel4 = 0b0100,
    Channel3 = 0b0011,
    Channel2 = 0b0010,
    Channel1 = 0b0001,
    Channel0 = 0b0000,
}

impl MuxSelection {
    pub fn try_from_channel_num(channel: u8) -> Option<Self> {
        Some(match channel {
            0 => MuxSelection::Channel0,
            1 => MuxSelection::Channel1,
            2 => MuxSelection::Channel2,
            3 => MuxSelection::Channel3,
            4 => MuxSelection::Channel4,
            5 => MuxSelection::Channel5,
            6 => MuxSelection::Channel6,
            7 => MuxSelection::Channel7,
            _ => return None,
        })
    }

    fn try_from_byte(byte: u8) -> Option<Self> {
        Some(match byte {
            0b1111 => MuxSelection::InternalVcm,
            0b1110 => MuxSelection::InternalTempSensorM,
            0b1101 => MuxSelection::InternalTempSensorP,
            0b1100 => MuxSelection::RefinM,
            0b1011 => MuxSelection::RefinP,
            0b1010 => MuxSelection::_Reserved,
            0b1001 => MuxSelection::AnalogVdd,
            0b1000 => MuxSelection::AnalogGnd,
            0b0111 => MuxSelection::Channel7,
            0b0110 => MuxSelection::Channel6,
            0b0101 => MuxSelection::Channel5,
            0b0100 => MuxSelection::Channel4,
            0b0011 => MuxSelection::Channel3,
            0b0010 => MuxSelection::Channel2,
            0b0001 => MuxSelection::Channel1,
            0b0000 => MuxSelection::Channel0,
            _ => return None,
        })
    }
}



#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RegisterMux{
    pub vin_p: MuxSelection,
    pub vin_m: MuxSelection,
}

impl Register for RegisterMux {
    fn address() -> u8 {
        0x06
    }

    fn size() -> usize {
        1
    }

    fn to_bytes(&self, out: &mut [u8]) {
        out[0] = (self.vin_p as u8) << 4 | (self.vin_m as u8);
    }
    
    fn try_from_bytes(bytes: &[u8]) -> Option<Self> {
        Some(RegisterMux {
            vin_p: MuxSelection::try_from_byte((bytes[0] & 0b1111_0000) >> 4)?,
            vin_m: MuxSelection::try_from_byte(bytes[0] & 0b0000_1111)?,
        })
    }
}

impl Default for RegisterMux {
    fn default() -> Self {
        Self {
            vin_p: MuxSelection::Channel0,
            vin_m: MuxSelection::Channel1,
        }
    }
}


impl From<Channel> for RegisterMux {
    fn from(value: Channel) -> Self {
        match value {
            Channel::SingleEnded0 => RegisterMux {
                vin_p: MuxSelection::Channel0,
                vin_m: MuxSelection::AnalogGnd,
            },
            Channel::SingleEnded1 => RegisterMux {
                vin_p: MuxSelection::Channel1,
                vin_m: MuxSelection::AnalogGnd,
            },
            Channel::SingleEnded2 => RegisterMux {
                vin_p: MuxSelection::Channel2,
                vin_m: MuxSelection::AnalogGnd,
            },
            Channel::SingleEnded3 => RegisterMux {
                vin_p: MuxSelection::Channel3,
                vin_m: MuxSelection::AnalogGnd,
            },
            Channel::SingleEnded4 => RegisterMux {
                vin_p: MuxSelection::Channel4,
                vin_m: MuxSelection::AnalogGnd,
            },
            Channel::SingleEnded5 => RegisterMux {
                vin_p: MuxSelection::Channel5,
                vin_m: MuxSelection::AnalogGnd,
            },
            Channel::SingleEnded6 => RegisterMux {
                vin_p: MuxSelection::Channel6,
                vin_m: MuxSelection::AnalogGnd,
            },
            Channel::SingleEnded7 => RegisterMux {
                vin_p: MuxSelection::Channel7,
                vin_m: MuxSelection::AnalogGnd,
            },
            Channel::Differential0_1 => RegisterMux {
                vin_p: MuxSelection::Channel0,
                vin_m: MuxSelection::Channel1,
            },
            Channel::Differential2_3 => RegisterMux {
                vin_p: MuxSelection::Channel2,
                vin_m: MuxSelection::Channel3,
            },
            Channel::Differential4_5 => RegisterMux {
                vin_p: MuxSelection::Channel4,
                vin_m: MuxSelection::Channel5,
            },
            Channel::Differential6_7 => RegisterMux {
                vin_p: MuxSelection::Channel6,
                vin_m: MuxSelection::Channel7,
            },
            Channel::Temp => RegisterMux {
                vin_p: MuxSelection::InternalTempSensorP,
                vin_m: MuxSelection::InternalTempSensorM,
            },
            Channel::AnalogVdd => RegisterMux {
                vin_p: MuxSelection::AnalogVdd,
                vin_m: MuxSelection::AnalogGnd,
            },
            Channel::Vcm => RegisterMux {
                vin_p: MuxSelection::InternalVcm,
                vin_m: MuxSelection::AnalogGnd,
            },
            Channel::Offset => RegisterMux {
                vin_p: MuxSelection::AnalogGnd,
                vin_m: MuxSelection::AnalogGnd,
            }
        }
    }
}






pub enum ScanDelayTime {
    Delay0 = 0b000,
    Delay8 = 0b001,
    Delay16 = 0b010,
    Delay32 = 0b011,
    Delay64 = 0b100,
    Delay128 = 0b101,
    Delay256 = 0b110,
    Delay512 = 0b111,
}

impl BitField for ScanDelayTime {
    const MASK: u8 = 0b1110_0000;
    const SHIFT: u8 = 5;

    fn as_u8_unshifted(self) -> u8 {
        self as u8
    }

    fn get(value: u8) -> Self {
        match (value & Self::MASK) >> Self::SHIFT {
            0b000 => ScanDelayTime::Delay0,
            0b001 => ScanDelayTime::Delay8,
            0b010 => ScanDelayTime::Delay16,
            0b011 => ScanDelayTime::Delay32,
            0b100 => ScanDelayTime::Delay64,
            0b101 => ScanDelayTime::Delay128,
            0b110 => ScanDelayTime::Delay256,
            0b111 => ScanDelayTime::Delay512,
            _ => unreachable!("Invalid scan delay time"),
        }
    }
}




pub enum Channel {
    SingleEnded0 = 0b0000,
    SingleEnded1 = 0b0001,
    SingleEnded2 = 0b0010,
    SingleEnded3 = 0b0011,
    SingleEnded4 = 0b0100,
    SingleEnded5 = 0b0101,
    SingleEnded6 = 0b0110,
    SingleEnded7 = 0b0111,
    Differential0_1 = 0b1000,
    Differential2_3 = 0b1001,
    Differential4_5 = 0b1010,
    Differential6_7 = 0b1011,
    Temp = 0b1100,
    AnalogVdd = 0b1101,
    Vcm = 0b1110,
    Offset = 0b1111,
}

impl Channel {
    pub fn try_from_channel_num_single_ended(channel: u8) -> Option<Self> {
        Some(match channel {
            0 => Channel::SingleEnded0,
            1 => Channel::SingleEnded1,
            2 => Channel::SingleEnded2,
            3 => Channel::SingleEnded3,
            4 => Channel::SingleEnded4,
            5 => Channel::SingleEnded5,
            6 => Channel::SingleEnded6,
            7 => Channel::SingleEnded7,
            _ => return None,
        })
    }

    fn bitflag(&self) -> u16 {
        match self {
            Self::SingleEnded0 => 0b0000_0000_0000_0001,
            Self::SingleEnded1 => 0b0000_0000_0000_0010,
            Self::SingleEnded2 => 0b0000_0000_0000_0100,
            Self::SingleEnded3 => 0b0000_0000_0000_1000,
            Self::SingleEnded4 => 0b0000_0000_0001_0000,
            Self::SingleEnded5 => 0b0000_0000_0010_0000,
            Self::SingleEnded6 => 0b0000_0000_0100_0000,
            Self::SingleEnded7 => 0b0000_0000_1000_0000,
            Self::Differential0_1 => 0b0000_0001_0000_0000,
            Self::Differential2_3 => 0b0000_0010_0000_0000,
            Self::Differential4_5 => 0b0000_0100_0000_0000,
            Self::Differential6_7 => 0b0000_1000_0000_0000,
            Self::Temp => 0b0001_0000_0000_0000,
            Self::AnalogVdd => 0b0010_0000_0000_0000,
            Self::Vcm => 0b0100_0000_0000_0000,
            Self::Offset => 0b1000_0000_0000_0000,
        }
    }
}


#[derive(Debug, Clone, Copy)]
pub struct RegisterScan([u8; 3]);

impl Register for RegisterScan {
    fn address() -> u8 {
        0x07
    }

    fn size() -> usize {
        3
    }

    fn to_bytes(&self, out: &mut [u8]) {
        out[0..3].copy_from_slice(&self.0)
    }
    
    fn try_from_bytes(bytes: &[u8]) -> Option<Self> where Self: Sized {
        Some(Self([bytes[0], bytes[1], bytes[2]]))
    }
}

impl Default for RegisterScan {
    fn default() -> Self {
        Self([0b0000_0000, 0b0000_0000, 0b0000_0000])
    }
}

impl RegisterScan {
    pub fn is_channel_enabled(&self, channel: Channel) -> bool {
        let bitflag = channel.bitflag().to_be_bytes();
        (&self.0[0] & bitflag[0] != 0) || (&self.0[1] & bitflag[1] != 0)
    }
    
    pub fn enable_channel(&mut self, channel: Channel) {
        let bitflag = channel.bitflag().to_be_bytes();
        self.0[0] |= bitflag[0];
        self.0[1] |= bitflag[1];
    }

    pub fn disable_channel(&mut self, channel: Channel) {
        let bitflag = channel.bitflag().to_be_bytes();
        self.0[0] &= !bitflag[0];
        self.0[1] &= !bitflag[1];
    }

    pub fn scan_delay_time(&self) -> ScanDelayTime {
        ScanDelayTime::get(self.0[2])
    }

    pub fn set_scan_delay_time(&mut self, scan_delay_time: ScanDelayTime) {
        scan_delay_time.set(&mut self.0[2]);
    }
}


#[derive(Debug, Clone, Copy)]
pub struct RegisterLock(u8);

impl Register for RegisterLock {
    fn address() -> u8 {
        0xD
    }

    fn size() -> usize {
        1
    }

    fn to_bytes(&self, out: &mut [u8]) {
        out[0] = self.0;
    }
    
    fn try_from_bytes(bytes: &[u8]) -> Option<Self> {
        Some(Self(bytes[0]))
    }
}

impl Default for RegisterLock {
    fn default() -> Self {
        RegisterLock(0xA5)
    }
}

impl RegisterLock {
    pub fn new_locked() -> Self {
        RegisterLock(0x0)
    }

    pub fn new_unlocked() -> Self {
        RegisterLock(0xA5)
    }

    pub fn lock(&mut self) {
        self.0 = 0x0;
    }

    pub fn unlock(&mut self) {
        self.0 = 0xA5;
    }

    pub fn is_locked(&self) -> bool {
        self.0 != 0xA5
    }
}


#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub config0: RegisterConfig0,
    pub config1: RegisterConfig1,
    pub config2: RegisterConfig2,
    pub config3: RegisterConfig3,
    pub irq: RegisterIRQ,
    pub mux: RegisterMux,
    pub scan: RegisterScan,

    timer: u32,
    offsetcal: u32,
    gaincal: u32,
    pub lock: RegisterLock,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            config0: RegisterConfig0::default(),
            config1: RegisterConfig1::default(),
            config2: RegisterConfig2::default(),
            config3: RegisterConfig3::default(),
            irq: RegisterIRQ::default(),
            mux: RegisterMux::default(),
            scan: RegisterScan::default(),
            timer: 0,
            offsetcal: 0,
            gaincal: 0,
            lock: RegisterLock::default(),
        }
    }

}

impl Config {
    /// Set the time interval between two consecutive scan cycles in multiples of the DMCLK period.
    /// Panics if the value is greater than 0xFFFFFF.
    pub fn set_scan_interval(&mut self, delay: u32) {
        assert!(delay <= 0xFFFFFF, "Timer value too large");
        self.timer = delay;
    }

    pub fn scan_interval(&self) -> u32 {
        self.timer
    }

    /// Set the offset calibration value.
    /// Panics if the value is greater than 0xFFFFFF.
    pub fn set_offset_calibration(&mut self, offset: u32) {
        assert!(offset <= 0xFFFFFF, "Offset calibration value too large");
        self.offsetcal = offset;
    }

    pub fn offset_calibration(&self) -> u32 {
        self.offsetcal
    }

    /// Set the gain calibration value.
    /// Panics if the value is greater than 0xFFFFFF.
    pub fn set_gain_calibration(&mut self, gain: u32) {
        assert!(gain <= 0xFFFFFF, "Gain calibration value too large");
        self.gaincal = gain;
    }

    pub fn gain_calibration(&self) -> u32 {
        self.gaincal
    }

    /// Converts to bytes starting from register address 0x01 up until address 0xD.
    pub(crate) fn to_bytes(&self, result: &mut [u8; 23]) {
        self.config0.to_bytes(&mut result[0..1]);
        self.config1.to_bytes(&mut result[1..2]);
        self.config2.to_bytes(&mut result[2..3]);
        self.config3.to_bytes(&mut result[3..4]);
        self.irq.to_bytes(&mut result[4..5]);
        self.mux.to_bytes(&mut result[5..6]);
        self.scan.to_bytes(&mut result[6..9]);
        result[9..12].copy_from_slice(&self.timer.to_be_bytes()[1..]);
        result[12..15].copy_from_slice(&self.offsetcal.to_be_bytes()[1..]);
        result[15..18].copy_from_slice(&self.gaincal.to_be_bytes()[1..]);
        self.lock.to_bytes(&mut result[22..23]);
    }

    pub(crate) fn from_bytes(bytes: &[u8; 23]) -> Option<Self> {
        Some(Config {
            config0: RegisterConfig0(bytes[0]),
            config1: RegisterConfig1(bytes[1]),
            config2: RegisterConfig2(bytes[2]),
            config3: RegisterConfig3(bytes[3]),
            irq: RegisterIRQ(bytes[4]),
            mux: RegisterMux::try_from_bytes(&bytes[5..6])?,
            scan: RegisterScan([bytes[6], bytes[7], bytes[8]]),
            timer: u32::from_be_bytes([0, bytes[9], bytes[10], bytes[11]]),
            offsetcal: u32::from_be_bytes([0, bytes[12], bytes[13], bytes[14]]),
            gaincal: u32::from_be_bytes([0, bytes[15], bytes[16], bytes[17]]),
            lock: RegisterLock(bytes[22]),
        })
    }


}


#[cfg(test)]
mod tests {
    use super::*;

    fn compare_scan_mux(scan_channel: super::Channel, mux_byte: u8) {
        let mut buf: [u8; 1] = [0];
        RegisterMux::from(scan_channel).to_bytes(&mut buf);
        assert_eq!(buf[0], mux_byte);
    }

    #[test]
    fn test_scan_to_mux() {
        // Taken from Table 5-15 in the datasheet
        compare_scan_mux(Channel::SingleEnded0, 0x08);
        compare_scan_mux(Channel::SingleEnded1, 0x18);
        compare_scan_mux(Channel::SingleEnded2, 0x28);
        compare_scan_mux(Channel::SingleEnded3, 0x38);
        compare_scan_mux(Channel::SingleEnded4, 0x48);
        compare_scan_mux(Channel::SingleEnded5, 0x58);
        compare_scan_mux(Channel::SingleEnded6, 0x68);
        compare_scan_mux(Channel::SingleEnded7, 0x78);
        compare_scan_mux(Channel::Differential0_1, 0x01);
        compare_scan_mux(Channel::Differential2_3, 0x23);
        compare_scan_mux(Channel::Differential4_5, 0x45);
        compare_scan_mux(Channel::Differential6_7, 0x67);
        compare_scan_mux(Channel::Temp, 0xDE);
        compare_scan_mux(Channel::AnalogVdd, 0x98);
        compare_scan_mux(Channel::Vcm, 0xF8);
        compare_scan_mux(Channel::Offset, 0x88);
    }
}