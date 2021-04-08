#include "fsm.h"
/*
enum year_state{
    SPRING = 1,
    SUMMER,
    AUTUMN,
    WINTER
};

enum year_event{
    EVENT1 = 1,
    EVENT2,
    EVENT3,
    EVENT4,
};
*/

/*
FsmTable_t test_table[] =
{
    //{到来的事件，当前的状态，将要要执行的函数，下一个状态}
    { EVENT1,  SPRING,    summer_thing,  SUMMER },
    { EVENT2,  SUMMER,    autumn_thing,  AUTUMN },
    { EVENT3,  AUTUMN,    winter_thing,  WINTER },
    { EVENT4,  WINTER,    spring_thing,  SPRING },
    //add your codes here
};
*/

/*
{
	FSM_t year_fsm;
	FSM_Regist(&year_fsm,year_table);
	year_fsm.curState = SPRING;
	year_fsm.size = sizeof(year_table)/sizeof(FsmTable_t);
}
{
	FSM_EventHandle(&year_fsm,EVENT2);
}
*/
