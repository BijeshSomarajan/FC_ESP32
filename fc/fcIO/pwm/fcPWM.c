#include "fcPWM.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "driver/mcpwm_prelude.h"
#include "soc/mcpwm_periph.h"
#include "fcLogger.h"
#include "mathUtil.h"

#if PWM_125_ENABLED == 1
#define PWM_BASE_PULSEWIDTH_US  125 //1000   // Minimum pulse width in microsecond
#define PWM_OUTPUT_MAX_PULSE_WIDTH_US 125 //1000
#define PWM_PERIOD_US  250 //250us  // 20000          // 20000 ticks, 20ms
#else
#define PWM_BASE_PULSEWIDTH_US  1000   // Minimum pulse width in microsecond
#define PWM_OUTPUT_MAX_PULSE_WIDTH_US 1000
#define PWM_PERIOD_US  20000          // 20000 ticks, 20ms
#endif

#define PWM_INPUT_MAX_PULSE_WIDTH_US 1000
#define PWM_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick

#define PWM_TIMER_COUNT  1
#define PWM_TIMER_OPERATOR_COUNT  2
#define PWM_TIMER_OPERATOR_GENERATOR_COUNT  2

uint8_t PWM_PORTS[] = { 25, 26, 32, 33 };

mcpwm_timer_handle_t timers[PWM_TIMER_COUNT];
mcpwm_oper_handle_t operators[PWM_TIMER_COUNT * PWM_TIMER_OPERATOR_COUNT];
mcpwm_cmpr_handle_t comparators[PWM_TIMER_COUNT * PWM_TIMER_OPERATOR_GENERATOR_COUNT * PWM_TIMER_OPERATOR_COUNT];
mcpwm_gen_handle_t generators[PWM_TIMER_COUNT * PWM_TIMER_OPERATOR_GENERATOR_COUNT * PWM_TIMER_OPERATOR_COUNT];



uint8_t initPWM(void) {
	char pwmLogBuffer[100];
	uint8_t genAndCompCount = 0;
	uint8_t operatorCount = 0;
	for (uint8_t timerCount = 0; timerCount < PWM_TIMER_COUNT; timerCount++) {
		mcpwm_timer_config_t timer_config = { .group_id = timerCount, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT, .resolution_hz = PWM_TIMEBASE_RESOLUTION_HZ, .period_ticks = PWM_PERIOD_US, .count_mode = MCPWM_TIMER_COUNT_MODE_UP, };
		esp_err_t err = mcpwm_new_timer(&timer_config, &timers[timerCount]);
		if (err != ESP_OK) {
			sprintf(pwmLogBuffer, "PWM , Timer:[%d] , Creation Failed!\n", timerCount);
			logString(pwmLogBuffer);
			return 0;
		}
		sprintf(pwmLogBuffer, "PWM , Timer:[%d] , Creation Success!\n", timerCount);
		logString(pwmLogBuffer);

		mcpwm_operator_config_t operator_config = { .group_id = timerCount, };

		for (uint8_t tempOperatorCount = 0; tempOperatorCount < PWM_TIMER_OPERATOR_COUNT; tempOperatorCount++) {
			//Creating Operator
			err = mcpwm_new_operator(&operator_config, &operators[operatorCount]);
			if (err != ESP_OK) {
				sprintf(pwmLogBuffer, "PWM , Timer[%d] , Operator:[%d] , Creation Failed!\n", timerCount, tempOperatorCount);
				logString(pwmLogBuffer);
				return 0;
			}
			sprintf(pwmLogBuffer, "PWM , Timer[%d] , Operator:[%d] , Creation Success!\n", timerCount, tempOperatorCount);
			logString(pwmLogBuffer);

			//Attaching Operator
			err = mcpwm_operator_connect_timer(operators[tempOperatorCount], timers[timerCount]);
			if (err != ESP_OK) {
				sprintf(pwmLogBuffer, "PWM , Timer[%d] , Operator[%i] , Association Failed!\n", timerCount, tempOperatorCount);
				logString(pwmLogBuffer);
				return 0;
			}
			sprintf(pwmLogBuffer, "PWM , Timer[%d] , Operator[%i] , Association Success!\n", timerCount, tempOperatorCount);
			logString(pwmLogBuffer);

			for (uint8_t tempGenAndCompCount = 0; tempGenAndCompCount < PWM_TIMER_OPERATOR_GENERATOR_COUNT; tempGenAndCompCount++) {

				//Creating Comparator
				mcpwm_comparator_config_t comparator_config = { .flags.update_cmp_on_tez = true, };
				err = mcpwm_new_comparator(operators[operatorCount], &comparator_config, &comparators[genAndCompCount]);
				if (err != ESP_OK) {
					sprintf(pwmLogBuffer, "PWM , Timer[%d] , Operator [%d] , Comparator[%i] , Creation Failed!\n", timerCount, tempOperatorCount, tempGenAndCompCount);
					logString(pwmLogBuffer);
					return 0;
				}
				sprintf(pwmLogBuffer, "PWM , Timer[%d] , Operator [%d] , Comparator[%i] , Creation Success!\n", timerCount, tempOperatorCount, tempGenAndCompCount);
				logString(pwmLogBuffer);

				//Creating generator
				mcpwm_generator_config_t generator_config = { .gen_gpio_num = PWM_PORTS[genAndCompCount] };
				err = mcpwm_new_generator(operators[operatorCount], &generator_config, &generators[genAndCompCount]);
				if (err != ESP_OK) {
					sprintf(pwmLogBuffer, "PWM , Timer[%d] ,  Operator [%d] , Generator[%i] , Creation Failed!\n", timerCount, tempOperatorCount, tempGenAndCompCount);
					logString(pwmLogBuffer);
					return 0;
				}
				sprintf(pwmLogBuffer, "PWM , Timer[%d] ,  Operator [%d] , Generator[%i] , Creation Success!\n", timerCount, tempOperatorCount, tempGenAndCompCount);
				logString(pwmLogBuffer);

				setPWMValue(genAndCompCount, 0);
				sprintf(pwmLogBuffer, "PWM , Timer[%d] ,  Operator [%d] , Generator[%i] , PWM[%d] , Set to 0 Success!\n", timerCount, tempOperatorCount, tempGenAndCompCount, genAndCompCount);
				logString(pwmLogBuffer);

				// Go high on counter empty
				err = mcpwm_generator_set_action_on_timer_event(generators[genAndCompCount], MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
				if (err != ESP_OK) {
					sprintf(pwmLogBuffer, "PWM , Timer[%d] ,  Operator [%d] , Generator[%i] , Empty Action Failed!\n", timerCount, tempOperatorCount, tempGenAndCompCount);
					logString(pwmLogBuffer);
					return 0;
				}
				sprintf(pwmLogBuffer, "PWM , Timer[%d] ,  Operator [%d] , Generator[%i] , Empty Action Success!\n", timerCount, tempOperatorCount, tempGenAndCompCount);

				// Go low on compare threshold
				err = mcpwm_generator_set_action_on_compare_event(generators[genAndCompCount], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[genAndCompCount], MCPWM_GEN_ACTION_LOW));
				if (err != ESP_OK) {
					sprintf(pwmLogBuffer, "PWM , Timer[%d] ,  Operator [%d] , Generator[%i] , TSH Action Failed!\n", timerCount, tempOperatorCount, tempGenAndCompCount);
					logString(pwmLogBuffer);
					return 0;
				}
				sprintf(pwmLogBuffer, "PWM , Timer[%d] ,  Operator [%d] , Generator[%i] , TSH Action Success!\n", timerCount, tempOperatorCount, tempGenAndCompCount);
				logString(pwmLogBuffer);

				genAndCompCount++;

			} //Generator and Comparator Loop

			operatorCount++;
		} //OperatorLoop

		err = mcpwm_timer_enable(timers[timerCount]);
		if (err != ESP_OK) {
			sprintf(pwmLogBuffer, "PWM , Timer[%i] , Enable Failed!\n", timerCount);
			logString(pwmLogBuffer);
			return 0;
		}
		sprintf(pwmLogBuffer, "PWM , Timer[%i] , Enable Success!\n", timerCount);
		logString(pwmLogBuffer);
	} //Timer Loop

	return 1;
}

uint8_t startPWMs() {
	char pwmLogBuffer[100];
	for (uint8_t timerCount = 0; timerCount < PWM_TIMER_COUNT; timerCount++) {
		esp_err_t err = mcpwm_timer_start_stop(timers[timerCount], MCPWM_TIMER_START_NO_STOP);
		if (err != ESP_OK) {
			sprintf(pwmLogBuffer, "PWM , Timer[%i] , Start Failed!\n", timerCount);
			logString(pwmLogBuffer);
			return 0;
		}
		sprintf(pwmLogBuffer, "PWM , Timer[%i] , Start Success!\n", timerCount);
		logString(pwmLogBuffer);
	}
	return 1;
}

uint8_t stopPWMs() {
	char pwmLogBuffer[100];
	for (uint8_t timerCount = 0; timerCount < PWM_CHANNEL_COUNT; timerCount++) {
		esp_err_t err = mcpwm_timer_start_stop(timers[timerCount], MCPWM_TIMER_STOP_EMPTY);
		if (err != ESP_OK) {
			sprintf(pwmLogBuffer, "PWM , Timer[%i] , Stop Failed!\n", timerCount);
			logString(pwmLogBuffer);
			return 0;
		}
		sprintf(pwmLogBuffer, "PWM , Timer[%i] , Stop Success!\n", timerCount);
		logString(pwmLogBuffer);
	}
	return 1;
}

void setPWMValue(uint8_t channel, uint16_t pwmValue) {
	uint16_t scaledPEMValue = PWM_BASE_PULSEWIDTH_US + mapToRange(pwmValue, 0, PWM_INPUT_MAX_PULSE_WIDTH_US, 0, PWM_OUTPUT_MAX_PULSE_WIDTH_US);
	mcpwm_comparator_set_compare_value(comparators[channel], scaledPEMValue);
}

