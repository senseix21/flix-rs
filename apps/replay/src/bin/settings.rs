use flight_config::{COUNT, PARAMETERS, RECORD_SIZE, Settings};
use std::{error::Error, io::Read, path::Path};
fn bounded(path: &Path, maximum: usize) -> Result<Vec<u8>, Box<dyn Error>> {
    let mut out = Vec::new();
    std::fs::File::open(path)?
        .take((maximum + 1) as u64)
        .read_to_end(&mut out)?;
    if out.len() > maximum {
        return Err("input exceeds limit".into());
    }
    Ok(out)
}
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = std::env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("defaults") if args.len()==3=>std::fs::write(&args[2],flight_config::encode(&Settings::default(),0).map_err(|e|format!("{e:?}"))?)?,
        Some("export") if args.len()==3=>{let bytes=bounded(Path::new(&args[2]),RECORD_SIZE)?;let (settings,sequence)=flight_config::decode(&bytes).map_err(|e|format!("{e:?}"))?;println!("# FLXC version=1 sequence={sequence}");for (i,p) in PARAMETERS.iter().enumerate() {println!("{}={}",p.name,settings.at(i).ok_or("index")?);}},
        Some("import") if args.len()==4=>{let bytes=bounded(Path::new(&args[2]),16384)?;let text=std::str::from_utf8(&bytes)?;let mut values=[None;COUNT];for line in text.lines().filter(|l|!l.is_empty() && !l.starts_with('#')) {let (name,value)=line.split_once('=').ok_or("name=value expected")?;let index=Settings::index(name).ok_or("unknown name")?;if values[index].is_some() {return Err("duplicate parameter".into());}values[index]=Some(value.parse::<f32>()?);}if values.iter().any(Option::is_none) {return Err("all 96 parameters required".into());}let settings=Settings::from_values(values.map(|v|v.expect("all present"))).map_err(|e|format!("{e:?}"))?;std::fs::write(&args[3],flight_config::encode(&settings,0).map_err(|e|format!("{e:?}"))?)?;},
        _=>return Err("usage: settings defaults record.bin | export record.bin | import values.txt record.bin".into())
    }
    Ok(())
}
