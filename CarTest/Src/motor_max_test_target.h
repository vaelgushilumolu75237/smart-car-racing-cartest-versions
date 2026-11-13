/* Included only by the explicitly selected, isolated motor test main. */
volatile motor_max_test_t car_motor_max_test;

static void motor_max_write_pwm(uint16_t duty)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!duty || duty > 1000u || !manual_run_button.run_request ||
        manual_run_button.stop_pending || car_control_state.fault_latched ||
        car_control_state.isr_stop_request ||
        (car_motor_max_test.phase != MOTOR_MAX_RAMP &&
         car_motor_max_test.phase != MOTOR_MAX_HOLD) ||
        (uint32_t)(HAL_GetTick() - car_motor_max_test.started_ms) >= MOTOR_MAX_RUN_MS) {
        stop_motor_outputs_hardware();
    } else {
        const uint32_t channels = TIM_CCER_CC2E | TIM_CCER_CC3E;
        /* Same proven forward pair as ordinary drive: left IN2, right IN1. */
        if (!motor_outputs_active) {
            TIM3->CCER = 0u; TIM3->CR1 &= ~TIM_CR1_CEN;
            TIM3->CCR1 = 0u; TIM3->CCR2 = 0u;
            TIM3->CCR3 = 0u; TIM3->CCR4 = 0u;
            TIM3->CNT = 0u; set_motor_gpio_low();
            set_motor_gpio_channels_af(channels);
        }
        TIM3->CCR1 = 0u; TIM3->CCR4 = 0u;
        TIM3->CCR2 = duty; TIM3->CCR3 = duty;
        TIM3->CR1 |= TIM_CR1_CEN;
        TIM3->CCER = channels;
        motor_outputs_active = 1u;
        applied_motor_duty_permille = duty;
        applied_motor_follow_left_permille = duty;
        applied_motor_follow_right_permille = duty;
        applied_motor_channel_mask = 6u;
        applied_follow_outputs = 0u;
        if (duty > car_bench_max_applied_duty_permille)
            car_bench_max_applied_duty_permille = duty;
    }
    __DSB();
    __set_PRIMASK(primask);
}

static void motor_max_run(void)
{
    uint32_t previous = HAL_GetTick();
    motor_max_init(&car_motor_max_test);
    if (car_iwdg_reset_seen || car_control_state.fault_latched)
        motor_max_stop(&car_motor_max_test, previous, MOTOR_MAX_STOP_RESET);
    for (;;) {
        uint32_t now = HAL_GetTick();
        uint32_t fault = 0u;
        uint32_t sample_status;
        uint8_t raw = pe7_is_high();
        car_main_loop_heartbeat_ms = now;
        (void)manual_button_sample(raw, now);
        if ((uint32_t)(now - previous) >= APP_MAIN_LOOP_GAP_LIMIT_MS) {
            fault = MOTOR_MAX_STOP_LOOP; car_fault = CAR_FAULT_LOOP_GAP;
        }
        previous = now;
        if (car_control_state.isr_stop_fault != CAR_FAULT_NONE ||
            car_control_state.fault_latched) {
            fault = MOTOR_MAX_STOP_HARDWARE;
        }
        if (!car_encoder_diag_update(&car_encoder_diag, TIM2->CNT, (uint16_t)TIM8->CNT)) {
            fault = MOTOR_MAX_STOP_ENCODER; car_fault = CAR_FAULT_ENCODER_INIT;
        }
        sample_status = car_speed_observer_update(&speed_observer, now,
            car_encoder_diag.left_raw_delta_sum, car_encoder_diag.right_raw_delta_sum,
            motor_outputs_active && applied_motor_duty_permille == 1000u,
            &speed_observer_result);
        if (sample_status >= CAR_SPEED_OBSERVER_STATUS_INVALID_CONFIG) {
            fault = MOTOR_MAX_STOP_ENCODER; car_fault = CAR_FAULT_ENCODER_INIT;
        }
        if (sample_status == CAR_SPEED_OBSERVER_STATUS_SAMPLE && speed_observer_result.valid)
            motor_max_sample(&car_motor_max_test, now, speed_observer_result.elapsed_ms,
                speed_observer_result.left_millirps, speed_observer_result.right_millirps,
                applied_motor_duty_permille);
        if (manual_run_button.start_pending) {
            car_manual_button_start_consumed(&manual_run_button);
            if (!fault) motor_max_start(&car_motor_max_test, now);
        }
        motor_max_tick(&car_motor_max_test, now, manual_run_button.run_request, fault);
        motor_max_write_pwm((uint16_t)car_motor_max_test.duty_permille);
        if (car_motor_max_test.phase == MOTOR_MAX_DONE || car_motor_max_test.phase == MOTOR_MAX_FAULT) {
            car_manual_button_cancel_request(&manual_run_button);
            car_manual_button_stop_consumed(&manual_run_button);
        }
        car_mode = car_motor_max_test.phase == MOTOR_MAX_READY ? CAR_MODE_WAIT_START_EDGE :
            car_motor_max_test.phase == MOTOR_MAX_DONE ? CAR_MODE_BENCH_COMPLETE :
            car_motor_max_test.phase == MOTOR_MAX_FAULT ? CAR_MODE_FAULT_LATCHED : CAR_MODE_BENCH_RUNNING;
        car_pe7_raw_high = raw;
        car_manual_run_request = manual_run_button.run_request;
        car_speed_left_millirps = speed_observer_result.valid ? speed_observer_result.left_millirps : 0;
        car_speed_right_millirps = speed_observer_result.valid ? speed_observer_result.right_millirps : 0;
        car_speed_left_rpm = speed_observer_result.valid ? speed_observer_result.left_rpm : 0;
        car_speed_right_rpm = speed_observer_result.valid ? speed_observer_result.right_rpm : 0;
        car_bench_ready = car_motor_max_test.phase == MOTOR_MAX_READY &&
                          manual_run_button.startup_release_confirmed;
        car_encoder_left_raw = car_encoder_diag.left_raw;
        car_encoder_right_raw = car_encoder_diag.right_raw;
        car_encoder_left_raw_delta_sum = car_encoder_diag.left_raw_delta_sum;
        car_encoder_right_raw_delta_sum = car_encoder_diag.right_raw_delta_sum;
        iwdg_refresh();
    }
}
