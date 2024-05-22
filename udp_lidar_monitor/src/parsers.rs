use nom::{
    bytes::complete::{tag, take_while_m_n},
    character::complete::{u32, u8},
    combinator::map_res,
    error::{self, Error},
    sequence::Tuple,
    IResult, Parser,
};

pub struct UDPAddress {
    pub a: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
}

pub struct UDPPort {
    pub port: u32,
}

fn u8_from_dec(input: &str) -> Result<u8, std::num::ParseIntError> {
    u8::from_str_radix(input, 10)
}
fn u32_from_dec(input: &str) -> Result<u32, std::num::ParseIntError> {
    u32::from_str_radix(input, 10)
}

fn is_digit(c: char) -> bool {
    c.is_digit(10)
}

fn parse_u8(input: &str) -> IResult<&str, u8> {
    map_res(take_while_m_n(3, 3, is_digit), u8_from_dec).parse(input)
}

fn parse_udp_address(input: &str) -> IResult<&str, UDPAddress> {
    let (input, (a, _, b, _, c, _, d)) =
        (u8, tag("."), u8, tag("."), u8, tag("."), u8).parse(input)?;
    Ok((input, UDPAddress { a, b, c, d }))
}
fn parse_udp_port(input: &str) -> IResult<&str, UDPPort> {
    let (input, port) = u32(input)?;
    if port > 10000 {
        return Err(nom::Err::Failure(Error::new(
            input,
            error::ErrorKind::TooLarge,
        )));
    }
    Ok((input, UDPPort { port }))
}
