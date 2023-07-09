#![feature(termination_trait)]
use std::ptr::read;

use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use log::*;
use esp_idf_sys::EspError;


use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::delay::BLOCK;
use esp_idf_hal::i2c::*;
use esp_idf_hal::ledc::*;

use esp_idf_hal::prelude::*;

use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::peripherals::Peripherals;
/*
pub struct E_Loads{
    peripherals:Peripherals,
}

impl E_Loads {
    pub fn new(){
        let peripherals=Peripherals::take().unwrap();
    }
    pub fn button()
}

pub struct Button<'a>{
    btn_a:PinDriver<'a, esp_idf_hal::gpio::Gpio39, esp_idf_hal::gpio::Input>,
    btn_b:PinDriver<'a, esp_idf_hal::gpio::Gpio38, esp_idf_hal::gpio::Input>,
    btn_c:PinDriver<'a, esp_idf_hal::gpio::Gpio37, esp_idf_hal::gpio::Input>,

}

impl Button<'_>{
    pub fn new(peripherals: &Peripherals)->Result<Button<'static>, EspError>{
        let peripherals = Peripherals::take().unwrap();

        let pin_a = &peripherals.pins.gpio39;
        let pin_b = &peripherals.pins.gpio38;
        let pin_c = &peripherals.pins.gpio37;

        let btn_a = PinDriver::input(peripherals.pins.gpio39).unwrap();
        let btn_b = PinDriver::input(peripherals.pins.gpio38).unwrap();
        let btn_c = PinDriver::input(peripherals.pins.gpio37).unwrap();

        Ok(Button {
            btn_a,
            btn_b,
            btn_c,
        })
    }
    pub fn pushed_a(&mut self)->bool{
        !self.btn_a.is_high()
    }
    pub fn pushed_b(&mut self)->bool{
        self.btn_b.is_high()
    }
    pub fn pushed_c(&mut self)->bool{
        self.btn_c.is_high()
    }
}

pub struct Dac<'a>{
    i2c:I2cDriver<'a>,
}

impl Dac<'_>{
    pub fn new(peripherals: &Peripherals)->Result<Dac, EspError>{

        let periph_i2c = peripherals.i2c0;
        let sda = peripherals.pins.gpio21;
        let scl = peripherals.pins.gpio22;
    
        let config = I2cConfig::new().baudrate((100*1000).into());
        let i2c = I2cDriver::new(periph_i2c, sda, scl, &config).unwrap();


        Ok(Dac {
            i2c,
        })
    }

    pub fn set_voltage(&mut self, voltage_v:f64){
        let dac_value = voltage_v as u16;
        const MCP4726A0T_ADDR: u8 = 0x60;

        //let n=1;
        //let v=0i16;
        let command_bit = 0b01100000;
        let v_ref_bit    = 0b00010000;
        let pd_bit      = 0b00000110;


        let data = [0x00,command_bit | v_ref_bit | pd_bit, (dac_value>>8 &0xff).try_into().unwrap(), (dac_value&0xff).try_into().unwrap()];
        self.i2c.write(MCP4726A0T_ADDR, &data, BLOCK);
    }
}
*/

const ADC_FSR:f32=4.096; //PGA 001 FSR=4.096

fn get_temperature_adc(adc:u16)->f32{
    let v_adc=adc as f32/2048.0*ADC_FSR;
    let r=1000.0;
    let vcc=3.3;
    let r_ntc=(r*vcc-v_adc*r)/v_adc;
    const B:f32=3984.0;
    const R25:f32=10000.0;
    const T25:f32=25.0+273.15;
    let r_r0=r_ntc/R25;
    1.0/(1.0/B*r_r0.log(std::f32::consts::E)+1.0/T25) - 273.15
}

fn get_current_adc(adc:u16)->f32{
    let v_adc=adc as f32/2048.0*ADC_FSR;
    v_adc/100.0/0.001
}

fn get_voltage_adc(adc:u16)->f32{
    let v_adc=adc as f32/2048.0*ADC_FSR;
    v_adc*(47.0+3.9)/3.3
}
////////////////////

fn get_temperature(i2c:&mut I2cDriver)->f32{
    let adc=read_adc(i2c, 1) as f32;
    let v_adc=adc/2048.0*ADC_FSR;

    let r_ntc=1000.0/v_adc-1000.0;

    const B:f32=3984.0;
    const R25:f32=10000.0;
    const T25:f32=25.0+273.15;
    let r_r0=r_ntc/R25;
    
    1.0/(1.0/B*r_r0.log(std::f32::consts::E)+1.0/T25) - 273.15

}

fn get_current(i2c:&mut I2cDriver)->f32{
    let adc=read_adc(i2c, 2) as f32;
    let v_adc=adc/2048.0*ADC_FSR;

    v_adc/100.0/0.001

}

fn get_voltage(i2c:&mut I2cDriver)->f32{
    let adc=read_adc(i2c, 3) as f32;
    let v_adc=adc/2048.0*ADC_FSR;

    v_adc*(47.0+3.9)/3.3

}

fn read_adc(i2c: &mut I2cDriver,ch: u16)->u16{
    const TLA202X_ADDR:u8=0b1001_000;
    const TLA202X_OS:u16=1;
    let TLA202X_MUX:u16=ch|0b100;
    const TLA202X_PGA:u16=0b001;
    const TLA202X_MODE:u16=1;
    const TLA202X_DR:u16=0b100;
    let data=TLA202X_OS<<15 | TLA202X_MUX<<12 | TLA202X_PGA<<9 | TLA202X_MODE<<8 | TLA202X_DR<<5;
    let data3byte =[0b01u8 ,((data>>8)&0xff) as u8 , ((data)&0xff) as u8];
    i2c.write(TLA202X_ADDR|0b0000_0000u8,&data3byte,      BLOCK);
    //i2c.write(TLA202X_ADDR|0b0000_0000u8,&[0b0000_0000u8],BLOCK);
    let mut buf=[0u8,0u8];

    i2c.write_read(TLA202X_ADDR|0b1000_0000u8, &[0b1u8],& mut buf, BLOCK);
    //info!("read:{},{}",buf[0],buf[1]);

    i2c.write_read(TLA202X_ADDR|0b1000_0000u8, &[0b0u8],& mut buf, BLOCK);

//    i2c.read(TLA202X_ADDR|0b1000_0000u8,&mut buf, BLOCK);
    //info!("ADC:{},{},{}",(buf[0] as u16)<<4| (buf[0] as u16)>>4, buf[0],buf[1]);
    (((buf[0] as u16)<<8| (buf[1] as u16)))>>4 as u16

    
}

fn set_current(i2c:&mut I2cDriver, current_ref:f64){
    let r_shunt=0.001;
    let amp=100.0;
    let v = current_ref*r_shunt*amp;
    set_voltage(i2c, v);
}
fn set_voltage(i2c:&mut I2cDriver, voltage_v:f64){
    let dac_value = (voltage_v / 3.3 * 4096.0) as u16;
    const MCP4726A0T_ADDR: u8 = 0x60;

    //let n=1;
    //let v=0i16;
    let command_bit = 0b01100000;
    let v_ref_bit    = 0b00010000;
    let pd_bit      = 0b00000110;


    let data = [0x00,command_bit | v_ref_bit | pd_bit, (dac_value>>8 &0xff).try_into().unwrap(), (dac_value&0xff).try_into().unwrap()];
    i2c.write(MCP4726A0T_ADDR, &data, BLOCK);
}

fn main()-> Result<(), EspError>{
    esp_idf_sys::link_patches();
    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();


//    let mut m5button = Button::new(&peripherals).unwrap();
//    let mut dac = Dac::new(&peripherals).unwrap();

    let peripherals = Peripherals::take().unwrap();
//////////////////////////////////
    let pin_a = peripherals.pins.gpio39;
    let pin_b = peripherals.pins.gpio38;
    let pin_c = peripherals.pins.gpio37;

    let btn_a = PinDriver::input(pin_a).unwrap();
    let btn_b = PinDriver::input(pin_b).unwrap();
    let btn_c = PinDriver::input(pin_c).unwrap();


    let mut rotary_sw = PinDriver::input(peripherals.pins.gpio13).unwrap();
    let mut RotaryENCA = PinDriver::input(peripherals.pins.gpio34).unwrap();
    let mut RotaryENCB = PinDriver::input(peripherals.pins.gpio0).unwrap();

    let led_r = PinDriver::output(peripherals.pins.gpio12).unwrap();
    let led_g = PinDriver::output(peripherals.pins.gpio5).unwrap();
    let led_b = PinDriver::output(peripherals.pins.gpio2).unwrap();

//////////////////////////////////

    let periph_i2c = peripherals.i2c0;
    let sda = peripherals.pins.gpio21;
    let scl = peripherals.pins.gpio22;

    let config = I2cConfig::new().baudrate((100.kHz()).into());
    let mut i2c = I2cDriver::new(periph_i2c, sda, scl, &config).unwrap();
////////////////////////////////////
    let timer_driver = LedcTimerDriver::new(peripherals.ledc.timer0, &esp_idf_hal::ledc::config::TimerConfig::default().frequency(25.kHz().into()))?;
    let mut driver = LedcDriver::new(peripherals.ledc.channel0,timer_driver, peripherals.pins.gpio15)?;

    let max_duty = driver.get_max_duty();
    let mut fun_duty=0.15;
    driver.set_duty( (max_duty as f32 * fun_duty) as u32)?;

    let mut i_ref=0.0;
    set_current(&mut i2c,i_ref);

    let mut pre_a = btn_a.is_low();
    let mut pre_b = btn_b.is_low();
    let mut pre_c = btn_c.is_low();
    let mut pre_rotary_sw = rotary_sw.is_high();


    loop{
        let adc3=read_adc(&mut i2c, 3);
        let adc2=read_adc(&mut i2c, 2);
        let adc1=read_adc(&mut i2c, 1);
        let adc0=read_adc(&mut i2c, 0);

        let current=get_current_adc(adc1);
        let voltage=get_voltage_adc(adc2);
        let temp=get_temperature_adc(adc0);
        // let current=get_current(&mut i2c);
        // let voltage=get_voltage(&mut i2c);
        // let temp=get_temperature(&mut i2c);
        info!("Vin:{:5.2} [V] Iref:{:5.2},I:{:5.2}, Temp:{:5.1} [degC] adc:{}",voltage,i_ref,current,temp,adc0);
        
        //info!("Iref:{:3.1}  ch0:{:4} ch1:{:4} ch2:{:4} ch3:{:4}",i_ref, adc0,adc1,adc2,adc3);
        
        if pre_a ==false && btn_a.is_low()==true {
            i_ref=i_ref+0.5;
            if i_ref >=5.0{
                i_ref=5.0
            }                
        }

        if pre_b ==false && btn_b.is_low()==true {
            fun_duty=0.15-fun_duty;
            driver.set_duty( (max_duty as f32 * fun_duty) as u32);
        }


        if pre_c ==false && btn_c.is_low()==true {
            i_ref=i_ref-0.5;
            if i_ref <0.0{
                i_ref=0.0
            }    
        }

        if pre_rotary_sw==false && rotary_sw.is_high(){
            set_current(&mut i2c,i_ref);
        }


        pre_a = btn_a.is_low();
        pre_b = btn_b.is_low();
        pre_c = btn_c.is_low();
        pre_rotary_sw = rotary_sw.is_high();

        FreeRtos::delay_ms(10);
    }

    Ok(())

}
