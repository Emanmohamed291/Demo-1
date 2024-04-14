/*
 * Runnables_List.h
 *
 *  Created on: Mar 14, 2024
 *      Author: Dell
 */

#ifndef RUNNABLES_LIST_H_
#define RUNNABLES_LIST_H_

/*** Runnable Names ***/
typedef enum {
	LCD_Runnable,
	SWITCH,
	StopWatch,
	APP_send,
	APP_receive,
	TimeDateUpdate,
	_Runnables_NUM
}RUNNABLES_t;

#endif /* RUNNABLES_LIST_H_ */
