#pragma once

template<class T>
class myvector {
public:
    myvector() {
        m_size = 0;
        m_buf = NULL;
    }
    ~myvector() {
        if (m_buf) {
            delete[] m_buf;
        }
    }
    void push_back(T v) {
        T* new_buf = new T[m_size+1];
        if (m_size > 0) {
            for(int i = 0; i < m_size; i++) {
                new_buf[i] = m_buf[i];
            }
            delete[] m_buf;
        }
        m_buf = new_buf;
        m_buf[m_size++] = v;
    }
    T& operator[](const int index) {
        return m_buf[index];
    }
    int size() { return m_size; }

private:
    int m_size;
    T *m_buf;
};

