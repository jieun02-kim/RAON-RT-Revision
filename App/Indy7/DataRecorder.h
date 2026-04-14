#include <atomic>

using namespace std;



struct ST_LOG_ENTRY {
    uint64_t timestamp_ns;
    double   q[6];        // 실제 관절 위치
    double   q_ref[6];    // 목표 관절 위치
    double   qd[6];       // 실제 관절 속도
    double   tau[6];      // 출력 토크
    double   tcp_x, tcp_y, tcp_z;  // TCP 포즈
    uint8_t  ctrl_mode;
};


template<typename T, size_t N>
class RingBuffer {
    private:
        T buf[N];
        atomic<size_t> head{0}, tail{0};
    public:
        bool push(const T& item) {           // RT 태스크에서 호출
            size_t h = head.load(memory_order_relaxed);
            size_t next = (h + 1) % N;
            if (next == tail.load(memory_order_acquire))
                return false;  // full, 드롭 (블로킹 없음)
            buf[h] = item;
            head.store(next, memory_order_release);
            return true;
        }
        bool pop(T& item) {                  // Logger 태스크에서 호출
            size_t t = tail.load(memory_order_relaxed);
            if (t == head.load(memory_order_acquire))
                return false;  // empty
            item = buf[t];
            tail.store((t + 1) % N, memory_order_release);
            return true;
        }
};

// 1kHz × 10분 버퍼 여유분
using LogRingBuffer = RingBuffer<ST_LOG_ENTRY, 65536>;