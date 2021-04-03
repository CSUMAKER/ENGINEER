#ifndef _FSM_H
#define _FSM_H

#include "makos_includes.h"

typedef struct FsmTable_s
{
    int event;   //事件
    int CurState;  //当前状态
    void (*eventActFun)();  //函数指针
    int NextState;  //下一个状态
}FsmTable_t;

/*状态机类型*/
typedef struct FSM_s{
    int curState;//当前状态
    FsmTable_t * pFsmTable;//状态表
    int size;//表的项数
}FSM_t;

/*状态机注册,给它一个状态表*/
void FSM_Regist(FSM_t* pFsm, FsmTable_t* pTable)
{
    pFsm->pFsmTable = pTable;
}

/*状态迁移*/
void FSM_StateTransfer(FSM_t* pFsm, int state)
{
    pFsm->curState = state;
}

/*事件处理*/
void FSM_EventHandle(FSM_t* pFsm, int event)
{
    FsmTable_t* pActTable = pFsm->pFsmTable;
    void (*eventActFun)() = NULL;  //函数指针初始化为空
    int NextState;
    int CurState = pFsm->curState;
    int g_max_num = pFsm->size;
    int flag = 0; //标识是否满足条件
    int i;

    /*获取当前动作函数*/
    for (i = 0; i<g_max_num; i++)
    {
        //当且仅当当前状态下来个指定的事件，我才执行它
        if (event == pActTable[i].event && CurState == pActTable[i].CurState)
        {
            flag = 1;
            eventActFun = pActTable[i].eventActFun;
            NextState = pActTable[i].NextState;
            break;
        }
    }


    if (flag) //如果满足条件了
    {
        /*动作执行*/
        if (eventActFun)
        {
            eventActFun();
        }

        //跳转到下一个状态
        FSM_StateTransfer(pFsm, NextState);
    }
    else
    {
        printf("there is no match\n");
    }
}

#endif
