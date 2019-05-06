#pragma once
const char* newGUID();

template <class T>
class SelfDeconstruction {
public:
	SelfDeconstruction(T *&p) {
		p = new T();
		m_pTr = p;
		m_len = 0;
	}
	SelfDeconstruction(T *&p,int len) {
		p = new T[len];
		m_pTr = p;
		m_len = len;
	}
	~SelfDeconstruction() {
		if (m_len)
			delete[]m_pTr;
		else
			delete m_pTr;
	}
private:
	T *m_pTr;
	int m_len;
};