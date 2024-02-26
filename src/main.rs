// Basado en el código de ejemplo del crate avr-hal para Arduino UNO
// https://github.com/Rahix/avr-hal/blob/main/examples/arduino-uno/src/bin/uno-hc-sr04.rs

#![no_std]
#![no_main]

use core::prelude::v1;

use arduino_hal::{hal::port::Dynamic, port::mode::Output, prelude::*, port::Pin};
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    let mut trig = pins.d12.into_output();
    let echo = pins.d13; // El pin es de entrada por defecto

    // Pines LEDs
    let mut pin_a0 = pins.d2.into_output();
    let mut pin_a1 = pins.d3.into_output();
    let mut pin_v0 = pins.d4.into_output();
    let mut pin_v1 = pins.d5.into_output();
    let mut pin_y0 = pins.d6.into_output();
    let mut pin_y1 = pins.d7.into_output();
    let mut pin_r0 = pins.d8.into_output();
    let mut pin_r1 = pins.d9.into_output();


    // Lista para iterar sobre los LEDs
    // Uso de Downgrading
    // https://rahix.github.io/avr-hal/avr_hal_generic/port/struct.Pin.html#downgrading
    let mut tira_led   = [
        &mut pin_a0.downgrade(),
        &mut pin_a1.downgrade(),
        &mut pin_v0.downgrade(),
        &mut pin_v1.downgrade(),
        &mut pin_y0.downgrade(),
        &mut pin_y1.downgrade(),
        &mut pin_r0.downgrade(),
        &mut pin_r1.downgrade(),
        
    ];

    // Prueba, apaga los LEDS
    apagar_leds(&mut tira_led);
    arduino_hal::delay_ms(1000);
    // Prueba para encender los LEDs
    encender_leds(&mut tira_led, 8_usize);
    arduino_hal::delay_ms(1000);

    // Arrancando e inicializando el temporizador con preescalado 64.
    // da una cuenta de reloj cada 4 µs.
    // como el tamaño del registro de reloj es de 16 bits, el temporizador se llena cada
    // 1/(16e6/64)*2^16 ≈ 260 ms
    let timer1 = dp.TC1;
    timer1.tccr1b.write(|w| w.cs1().prescale_64());


    'outer: loop {
        // Se reinicializa el timer con cero.
        timer1.tcnt1.write(|w| w.bits(0));

        // El trigger debe ser puesto en HIGH por 10µs, de acuerdo a la ficha técnica del sensor
        trig.set_high();
        arduino_hal::delay_us(10);
        trig.set_low();

        while echo.is_low() {
            // salir del bucle si el temporizador ha llegado a 200 ms.
            // 0.2s/4µs = 50000
            if timer1.tcnt1.read().bits() >= 50000 {
                // Si no se detecta un obstáculo, regresar al inicio del loop
                ufmt::uwriteln!(
                    &mut serial,
                    "No se detectó nada...\r"
                )
                .unwrap();
                continue 'outer;
            }
        }
        // Restart el timer
        timer1.tcnt1.write(|w| w.bits(0));

        // Esperar el eco
        while echo.is_high() {}

        // 1 count == 4 µs, por lo que el valor se multiplica por 4.
        // 1/58 ≈ (34000 cm/s) * 1µs / 2
        // cuando no se detecta ningún objeto, en lugar de mantener el pin eco completamente bajo,
        // algunos sensores etiquetados HC-SR04 mantienen el pin eco en estado alto durante mucho tiempo,
        // desbordando así el valor de u16 al multiplicar el valor de timer1 por 4.
        // ¡el desbordamiento durante el tiempo de ejecución causa pánico! por lo que debe ser manejadocial
        let temp_timer = timer1.tcnt1.read().bits().saturating_mul(4);
        let value = match temp_timer {
            u16::MAX => {
                ufmt::uwriteln!(
                    &mut serial,
                    "No se detectó nada...\r"
                )
                .unwrap();
                continue 'outer;
            }
            _ => temp_timer / 58,
        };

        // Await 100 ms before sending the next trig
        // 0.1s/4µs = 25000
        while timer1.tcnt1.read().bits() < 25000 {}

        let maximo: usize = (8 - (value * 8) / 170).into();
        apagar_leds(&mut tira_led);
        encender_leds(&mut tira_led, maximo);

        ufmt::uwriteln!(
            &mut serial,
            "Detectando el objetivo {} cms \r",
            value
        )
        .unwrap();
    }
}

fn apagar_leds(tira: &mut [&mut Pin<Output, Dynamic>; 8] ) {
    for i in 0..tira.len() {
        tira[i].set_low();
    }
}

fn encender_leds(tira: &mut [&mut Pin<Output, Dynamic>; 8], maximo: usize ) {


    for i in 0..maximo{
        tira[i].set_high();
    }
}