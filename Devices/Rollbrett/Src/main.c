void device_specific(){
    int turn = (adc_buffer.l_rx2 - ADC_MID) / 8;
    int speed = (adc_buffer.l_tx2 - ADC_MID) / 4;

    if (ABS(turn) < 4) {
      turn = 0;
    } else {
      turn -= 4 * SIGN(turn);
    }

    if (ABS(speed) < 5) {
      speed = 0;
    }

    set_throttle(speed + turn, speed - turn);
      // (adc_buffer.l_tx2-ADC_MID) / 2 + (adc_buffer.l_rx2-ADC_MID) / 2,
      // (adc_buffer.l_tx2-ADC_MID) / 2 - (adc_buffer.l_rx2-ADC_MID) / 2);
}