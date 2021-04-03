#ifndef _FSM_H
#define _FSM_H

#include "makos_includes.h"

typedef struct FsmTable_s
{
    int event;   //�¼�
    int CurState;  //��ǰ״̬
    void (*eventActFun)();  //����ָ��
    int NextState;  //��һ��״̬
}FsmTable_t;

/*״̬������*/
typedef struct FSM_s{
    int curState;//��ǰ״̬
    FsmTable_t * pFsmTable;//״̬��
    int size;//�������
}FSM_t;

/*״̬��ע��,����һ��״̬��*/
void FSM_Regist(FSM_t* pFsm, FsmTable_t* pTable)
{
    pFsm->pFsmTable = pTable;
}

/*״̬Ǩ��*/
void FSM_StateTransfer(FSM_t* pFsm, int state)
{
    pFsm->curState = state;
}

/*�¼�����*/
void FSM_EventHandle(FSM_t* pFsm, int event)
{
    FsmTable_t* pActTable = pFsm->pFsmTable;
    void (*eventActFun)() = NULL;  //����ָ���ʼ��Ϊ��
    int NextState;
    int CurState = pFsm->curState;
    int g_max_num = pFsm->size;
    int flag = 0; //��ʶ�Ƿ���������
    int i;

    /*��ȡ��ǰ��������*/
    for (i = 0; i<g_max_num; i++)
    {
        //���ҽ�����ǰ״̬������ָ�����¼����Ҳ�ִ����
        if (event == pActTable[i].event && CurState == pActTable[i].CurState)
        {
            flag = 1;
            eventActFun = pActTable[i].eventActFun;
            NextState = pActTable[i].NextState;
            break;
        }
    }


    if (flag) //�������������
    {
        /*����ִ��*/
        if (eventActFun)
        {
            eventActFun();
        }

        //��ת����һ��״̬
        FSM_StateTransfer(pFsm, NextState);
    }
    else
    {
        printf("there is no match\n");
    }
}

#endif
