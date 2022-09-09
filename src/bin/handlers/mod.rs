use bxcan::{Data, Frame, StandardId};
use canpsa::{aee2004, aee2010, Error as CanPsaError};
use core::result::Result;

pub enum HandlerDecision {
    Ok,
    Forward(Frame),
}

macro_rules! gen_direct_forwarding_handler_2004 {
    ($func: ident, $frame_id: ident) => {
        pub fn $func(raw_buf: &[u8]) -> Result<HandlerDecision, CanPsaError> {
            let wrapped_frame = aee2004::conf::$frame_id::Frame::new_checked(raw_buf)?;
            let repr = aee2004::conf::$frame_id::Repr::parse(&wrapped_frame)?;

            let repr_2010 = aee2010::infodiv::$frame_id::Repr::from(&repr);
            let mut emit_buf = [0; aee2010::infodiv::$frame_id::FRAME_LEN];
            let mut wrapped_emit_buf =
                aee2010::infodiv::$frame_id::Frame::new_unchecked(&mut emit_buf);
            repr_2010.emit(&mut wrapped_emit_buf);

            let can_frame = Frame::new_data(
                StandardId::new(aee2010::infodiv::$frame_id::FRAME_ID).unwrap(),
                Data::new(&emit_buf).unwrap(),
            );

            Ok(HandlerDecision::Forward(can_frame))
        }
    };
}

macro_rules! gen_direct_forwarding_handler_2010 {
    ($func: ident, $frame_id: ident) => {
        pub fn $func(raw_buf: &[u8]) -> Result<HandlerDecision, CanPsaError> {
            let wrapped_frame = aee2010::infodiv::$frame_id::Frame::new_checked(raw_buf)?;
            let repr = aee2010::infodiv::$frame_id::Repr::parse(&wrapped_frame)?;

            let repr_2004 = aee2004::conf::$frame_id::Repr::from(&repr);
            let mut emit_buf = [0; aee2004::conf::$frame_id::FRAME_LEN];
            let mut wrapped_emit_buf =
                aee2004::conf::$frame_id::Frame::new_unchecked(&mut emit_buf);
            repr_2004.emit(&mut wrapped_emit_buf);

            let can_frame = Frame::new_data(
                StandardId::new(aee2004::conf::$frame_id::FRAME_ID).unwrap(),
                Data::new(&emit_buf).unwrap(),
            );

            Ok(HandlerDecision::Forward(can_frame))
        }
    };
}

macro_rules! gen_transform_forwarding_handler_2004 {
    ($func: ident, $frame_id_in: ident, $frame_id_out: ident) => {
        pub fn $func(raw_buf: &[u8]) -> Result<HandlerDecision, CanPsaError> {
            let wrapped_frame = aee2004::conf::$frame_id_in::Frame::new_checked(raw_buf)?;
            let repr = aee2004::conf::$frame_id_in::Repr::parse(&wrapped_frame)?;

            let repr_2010 = aee2010::infodiv::$frame_id_out::Repr::from(&repr);
            let mut emit_buf = [0; aee2010::infodiv::$frame_id_out::FRAME_LEN];
            let mut wrapped_emit_buf =
                aee2010::infodiv::$frame_id_out::Frame::new_unchecked(&mut emit_buf);
            repr_2010.emit(&mut wrapped_emit_buf);

            let can_frame = Frame::new_data(
                StandardId::new(aee2010::infodiv::$frame_id_out::FRAME_ID).unwrap(),
                Data::new(&emit_buf).unwrap(),
            );

            Ok(HandlerDecision::Forward(can_frame))
        }
    };
}

macro_rules! gen_parsing_handler_2004 {
    ($func: ident, $frame_id: ident) => {
        pub fn $func(raw_buf: &[u8]) -> Result<aee2004::conf::$frame_id::Repr, CanPsaError> {
            let wrapped_frame = aee2004::conf::$frame_id::Frame::new_checked(raw_buf)?;
            let repr = aee2004::conf::$frame_id::Repr::parse(&wrapped_frame)?;
            Ok(repr)
        }
    };
}

macro_rules! gen_parsing_handler_2010 {
    ($func: ident, $frame_id: ident) => {
        pub fn $func(raw_buf: &[u8]) -> Result<aee2010::infodiv::$frame_id::Repr, CanPsaError> {
            let wrapped_frame = aee2010::infodiv::$frame_id::Frame::new_checked(raw_buf)?;
            let repr = aee2010::infodiv::$frame_id::Repr::parse(&wrapped_frame)?;
            Ok(repr)
        }
    };
}

macro_rules! gen_encode_and_forward_handler_2004 {
    ($func: ident, $frame_id: ident) => {
        pub fn $func(
            repr_2004: &aee2004::conf::$frame_id::Repr,
        ) -> Result<HandlerDecision, CanPsaError> {
            let mut emit_buf = [0; aee2004::conf::$frame_id::FRAME_LEN];
            let mut wrapped_emit_buf =
                aee2004::conf::$frame_id::Frame::new_unchecked(&mut emit_buf);
            repr_2004.emit(&mut wrapped_emit_buf);

            let can_frame = Frame::new_data(
                StandardId::new(aee2004::conf::$frame_id::FRAME_ID).unwrap(),
                Data::new(&emit_buf).unwrap(),
            );

            Ok(HandlerDecision::Forward(can_frame))
        }
    };
}

macro_rules! gen_encode_and_forward_handler_2010 {
    ($func: ident, $frame_id: ident) => {
        pub fn $func(
            repr_2010: &aee2010::infodiv::$frame_id::Repr,
        ) -> Result<HandlerDecision, CanPsaError> {
            let mut emit_buf = [0; aee2010::infodiv::$frame_id::FRAME_LEN];
            let mut wrapped_emit_buf =
                aee2010::infodiv::$frame_id::Frame::new_unchecked(&mut emit_buf);
            repr_2010.emit(&mut wrapped_emit_buf);

            let can_frame = Frame::new_data(
                StandardId::new(aee2010::infodiv::$frame_id::FRAME_ID).unwrap(),
                Data::new(&emit_buf).unwrap(),
            );

            Ok(HandlerDecision::Forward(can_frame))
        }
    };
}

macro_rules! gen_encode_and_forward_handler_with_checksum_2010 {
    ($func: ident, $frame_id: ident) => {
        pub fn $func(
            repr_2010: &aee2010::infodiv::$frame_id::Repr,
            chk_cnt: &mut u8,
        ) -> Result<HandlerDecision, CanPsaError> {
            let mut emit_buf = [0; aee2010::infodiv::$frame_id::FRAME_LEN];
            let mut wrapped_emit_buf =
                aee2010::infodiv::$frame_id::Frame::new_unchecked(&mut emit_buf);
            repr_2010.emit(&mut wrapped_emit_buf);
            wrapped_emit_buf.fill_checksum(chk_cnt);

            let can_frame = Frame::new_data(
                StandardId::new(aee2010::infodiv::$frame_id::FRAME_ID).unwrap(),
                Data::new(&emit_buf).unwrap(),
            );

            Ok(HandlerDecision::Forward(can_frame))
        }
    };
}

gen_direct_forwarding_handler_2004!(handle_2004_x128, x128);
gen_direct_forwarding_handler_2004!(handle_2004_x168, x168);
gen_direct_forwarding_handler_2004!(handle_2004_x1e1, x1e1);
gen_direct_forwarding_handler_2004!(handle_2004_x261, x261);
gen_direct_forwarding_handler_2004!(handle_2004_x2a1, x2a1);
gen_direct_forwarding_handler_2004!(handle_2004_x361, x361);

gen_direct_forwarding_handler_2010!(handle_2010_x1e5, x1e5);

gen_transform_forwarding_handler_2004!(handle_2004_x1a8, x1a8, x228);
gen_transform_forwarding_handler_2004!(handle_2004_x1d0, x1d0, x350);
gen_transform_forwarding_handler_2004!(handle_2004_x3a7, x3a7, x3e7);

gen_parsing_handler_2004!(parse_2004_x036, x036);
gen_parsing_handler_2004!(parse_2004_x0e6, x0e6);
gen_parsing_handler_2004!(parse_2004_x0f6, x0f6);
gen_parsing_handler_2004!(parse_2004_x220, x220);
gen_parsing_handler_2004!(parse_2004_x227, x227);
gen_parsing_handler_2004!(parse_2004_x260, x260);

gen_parsing_handler_2010!(parse_2010_x15b, x15b);
gen_parsing_handler_2010!(parse_2010_x1a9, x1a9);
gen_parsing_handler_2010!(parse_2010_x39b, x39b);

gen_encode_and_forward_handler_2004!(encode_2004_x167, x167);
gen_encode_and_forward_handler_2010!(encode_2010_x260, x260);

gen_encode_and_forward_handler_with_checksum_2010!(encode_2010_x0e6, x0e6);
