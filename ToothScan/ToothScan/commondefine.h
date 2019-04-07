#pragma once
#include <iostream>
#include <memory>
#define PARAMDEFINE(type , prefix,prarm_name)	private:\
										type m_##prefix##prarm_name;\
										public:\
										void Set_##prarm_name(type v_##prarm_name){m_##prefix##prarm_name = v_##prarm_name;};\
										type Get_##prarm_name(){return m_##prefix##prarm_name;}

#define SharedPtr(ClassName) typedef shared_ptr<ClassName> p##ClassName;