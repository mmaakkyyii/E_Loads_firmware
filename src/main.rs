#![feature(termination_trait)]
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
    let mut fun_duty=0.1;
    driver.set_duty( (max_duty as f32 * fun_duty) as u32);


    let mut i_ref=0.0;
    loop{
        info!("Iref:{}  A:{} B:{} C:{}",i_ref, btn_a.is_high(), btn_b.is_high(), btn_c.is_high());
        if btn_a.is_high()==false {
            i_ref=i_ref+0.5;
            set_current(&mut i2c,i_ref);
            if i_ref >=2.0{
                i_ref=2.0
            }                
        }

        if btn_b.is_high()==false{
            i_ref=i_ref-0.5;
            set_current(&mut i2c,i_ref);
            if i_ref <0.0{
                i_ref=0.0
            }    
        }

        FreeRtos::delay_ms(1000);
    }

    Ok(())

}


//*/
/*
#![feature(termination_trait)]
use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use log::*;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::delay::BLOCK;
use esp_idf_hal::gpio::Level;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::gpio::Pull;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::ledc::*;

use esp_idf_hal::i2c::*;

use esp_idf_sys::EspError;

fn read_ADC(i2c: &mut I2cDriver,ch: u16)->u32{
    const TLA202x_ADDR:u8=0b1001_000;
    const TLA202x_OS:u16=1;
    let TLA202x_MUX:u16=ch+0b100;
    const TLA202x_PGA:u16=0b001;
    const TLA202x_MODE:u16=1;
    const TLA202x_DR:u16=0b100;
    let data=TLA202x_OS<<15 | TLA202x_MUX<<12 | TLA202x_PGA<<9 | TLA202x_MODE<<8 | TLA202x_DR<<5;
    let data2byte =[0b01u8 ,((data>>8)&0xff) as u8 , ((data)&0xff) as u8];
    i2c.write(TLA202x_ADDR|0b0000_0000u8,&data2byte,BLOCK);
    let mut buf=[0u8,0u8];
    i2c.write(TLA202x_ADDR|0b0000_0000u8,&[0b0000_0000u8],BLOCK);
   // i2c.write_read(TLA202x_ADDR|0b1000_0000u8, &[0b1u8],& mut buf, BLOCK);
    i2c.read(TLA202x_ADDR|0b1000_0000u8,&mut buf, BLOCK);
    //info!("ADC:{},{},{}",(buf[0] as u16)<<4| (buf[0] as u16)>>4, buf[0],buf[1]);
    ((buf[0] as u16)<<4| (buf[0] as u16)>>4) as u32

    
}
fn main() -> Result<(), EspError>{
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_sys::link_patches();
    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let mut BtnA = PinDriver::input(peripherals.pins.gpio39).unwrap();
    let mut BtnB = PinDriver::input(peripherals.pins.gpio38).unwrap();
    let mut BtnC = PinDriver::input(peripherals.pins.gpio37).unwrap();

    let mut RotarySW = PinDriver::input(peripherals.pins.gpio13).unwrap();
    let mut RotaryENCA = PinDriver::input(peripherals.pins.gpio34).unwrap();
    let mut RotaryENCB = PinDriver::input(peripherals.pins.gpio0).unwrap();

    let mut ledR = PinDriver::output(peripherals.pins.gpio12).unwrap();
    let mut ledG = PinDriver::output(peripherals.pins.gpio5).unwrap();
    let mut ledB = PinDriver::output(peripherals.pins.gpio2).unwrap();

//    let mut fan = PinDriver::output(peripherals.pins.gpio15).unwrap();
//    fan.set_low();

    let timer_driver = LedcTimerDriver::new(peripherals.ledc.timer0, &esp_idf_hal::ledc::config::TimerConfig::default().frequency(25.kHz().into()))?;
    let mut driver = LedcDriver::new(peripherals.ledc.channel0,timer_driver, peripherals.pins.gpio15)?;

    let max_duty = driver.get_max_duty();
    driver.set_duty(0);
    let mut fun_duty=0.0;

    let mut preA=RotaryENCA.is_high();
    let mut preB=RotaryENCB.is_high();


    let i2c = peripherals.i2c0;
    let sda = peripherals.pins.gpio21;
    let scl = peripherals.pins.gpio22;

    const MCP4726A0T_ADDR: u8 = 0x60;
    let config = I2cConfig::new().baudrate(100.kHz().into());
    let mut i2c = I2cDriver::new(i2c, sda, scl, &config).unwrap();

    let mut n=1;
    let mut v=0i16;
    let command_bit = 0b01100000;
    let Vref_bit    = 0b00010000;
    let PD_bit      = 0b00000110;
    while true {
        let mut data=0b01000000;
        
      //  info!("Set {} fun {} ADC {}, {}, {}", v, fun_duty,read_ADC(&mut i2c,0),read_ADC(&mut i2c,1),read_ADC(&mut i2c,2));
        n+=1;
        //v+=1023;
/*
        if preA{
            if RotaryENCA.is_low(){
                if RotaryENCB.is_high(){
                    fun_duty=fun_duty-0.05;
                } else {
                    fun_duty=fun_duty+0.05;
                }
            }
        }else{
            if RotaryENCA.is_high(){
                if RotaryENCB.is_high(){
                    fun_duty=fun_duty+0.05;
                } else {
                    fun_duty=fun_duty-0.05;
                }
            }
        }
        if preB{
            if RotaryENCB.is_low(){
                if RotaryENCA.is_high(){
                    fun_duty=fun_duty-0.05;
                } else {
                    fun_duty=fun_duty+0.05;
                }
            }
        }else{
            if RotaryENCB.is_high(){
                if RotaryENCA.is_high(){
                    fun_duty=fun_duty+0.05;
                } else {
                    fun_duty=fun_duty-0.05;
                }
            }
        }

        if fun_duty>1.0{
            fun_duty=1.0;
        }
        if fun_duty<0.0{
            fun_duty=0.0;
        }
        preA=RotaryENCA.is_high();
        preB=RotaryENCB.is_high();
    
        fun_duty=0.15;
        driver.set_duty( (max_duty as f32 * fun_duty) as u32);

        if RotarySW.is_high(){
            driver.set_duty( (max_duty as f32 * fun_duty) as u32);
        } else {
  //          driver.set_duty(0);
//            fan.set_low();
        }
        */
        if BtnA.is_high() {
        //    ledR.set_high();
        } else {
            v+=1;
        //    ledR.set_low();
        }

        if BtnB.is_high() {
        //    ledG.set_high();
        } else {
            v-=1;

        //    ledG.set_low();
        }

        if BtnC.is_high() {
        //    ledB.set_high();
        } else {
            ledB.set_high();
        //    i2c.write(MCP4726A0T_ADDR, &[0x00,command_bit | Vref_bit | PD_bit, (v>>8 &0xff).try_into().unwrap(), (v&0xff).try_into().unwrap()],BLOCK);
        }
        if v > 4096{
            v=4095;
        }
        if v<0{
            v=0;
        }
        FreeRtos::delay_ms(10);
        // LEDG.set_high()?;
        // FreeRtos::delay_ms(500);
        // LEDB.set_high()?;
        // FreeRtos::delay_ms(500);

        // LEDR.set_low();
        // FreeRtos::delay_ms(500);
        // LEDG.set_low()?;
        // FreeRtos::delay_ms(500);
        // LEDB.set_low()?;
        // FreeRtos::delay_ms(500);

    }

    Ok(())  

    
}
*/