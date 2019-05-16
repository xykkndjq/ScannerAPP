#pragma once
#include <iostream>
#include <memory>
#define PARAMDEFINE(type , prefix,prarm_name)	private:\
										type m_##prefix##prarm_name;\
										public:\
										void Set_##prarm_name(type v_##prarm_name){m_##prefix##prarm_name = v_##prarm_name;};\
										type Get_##prarm_name(){return m_##prefix##prarm_name;}

#define SharedPtr(ClassName) typedef shared_ptr<ClassName> p##ClassName;


#define DECLARE_SINGLEINSTANCE(ClassName) \
    private:\
        static ClassName* singleInstance;\
    public:\
        static ClassName* shareInstance()\
        {\
            if(NULL == singleInstance) singleInstance = new ClassName();\
            return singleInstance;\
        }\
    private:\
        class CGarbo\
        {\
            public:\
            ~CGarbo()\
            {if( ClassName::singleInstance )delete ClassName::singleInstance;}\
        };\
        static CGarbo Garbo;
//
#define DEFINITION_SINGLEINSTANCE(ClassName)\
    ClassName* ClassName::singleInstance = NULL;