#ifndef APP_TASK_CONFIG_H
#define APP_TASK_CONFIG_H

#define APP_TASK_BASKET 1  // 抓篮：左右电机+激光+夹爪；basket_request=1启动
#define APP_TASK_LIFT   2  // 升降：0x06电机；先自动执行上电预定位，再等待lift_request
#define APP_TASK_COMBINED 3 // 联调：原位推出/返回→升降到位→抓篮

/* 只改此处选择任务，重新编译下载后生效；不是运行时切换。 */
#ifndef APP_TASK_MODE
#define APP_TASK_MODE APP_TASK_LIFT
#endif

#if APP_TASK_MODE != APP_TASK_BASKET && APP_TASK_MODE != APP_TASK_LIFT && APP_TASK_MODE != APP_TASK_COMBINED
#error "APP_TASK_MODE must be APP_TASK_BASKET or APP_TASK_LIFT"
#endif

#endif
