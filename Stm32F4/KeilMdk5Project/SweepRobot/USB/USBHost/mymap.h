#pragma once

template<class K,class T>
class mymap {
    struct mypair {
        K first;
        T second;
    };
public:
    mymap() {
        m_size = 0;
    }
    T& operator[](const K& key) {
        int it;
        if (count(key) == 0) {
            it = insert(key, 0);
        } else {
            it = find(key);
        }
        return m_buf[it].second;
    }
    bool empty() { return m_size == 0 ? true : false; }
    int size() { return m_size; }
    void clear() { m_size = 0; }
    int count(K key) {
        for(int i = 0; i < m_size; i++) {
            if (m_buf[i].first == key) {
                return 1;
            }
        }
        return 0;
    }

private:
    int find(K key) {
        for(int i = 0; i < m_size; i++) {
            if (m_buf[i].first == key) {
                return i;
            }
        }
        return -1;
    }
    int insert(K key, T value) {
        int it = find(key);
        if (it != -1) {
            m_buf[it].second = value;
            return it;
        }
        mypair* new_buf = new mypair[m_size+1];
        if (m_size > 0) {
            for(int i = 0; i < m_size; i++) {
                new_buf[i] = m_buf[i];
            }
            delete[] m_buf;
        }
        m_buf = new_buf;
        it = m_size++;
        m_buf[it].first = key;
        m_buf[it].second = value;
        return it;
    }

    int m_size;
    mypair *m_buf;
};

