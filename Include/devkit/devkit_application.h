 /* Copyright (c) 2013 Belwith Products. All Rights Reserved.
 *
 */

#ifndef SMARTLOCK_APPLICATION_H
#define SMARTLOCK_APPLICATION_H

#define KEY_DETECT_SW_CH						2


/** initialize development_kit
*/
void development_kit_gpio_init(void);

/** function which will perform activity of application at end of 1 second
*/
void app_timer_activity(void);

#endif

