#ifndef _TOPIC_
#define _TOPIC_

#include <deque>
#include <vector>
#include <chrono>
#include <iostream>
#include <utility>
#include <mutex> 

template<class Struct>
class Topic{
    public:
    Topic(std::chrono::high_resolution_clock::time_point init_timepoint=std::chrono::high_resolution_clock::now(), size_t max_size=1);
    ~Topic();
    void update(Struct content);
    bool latest(int64_t& timestamp, Struct& content);
    void recent(std::vector< std::pair<int64_t, Struct> >& topic_vector, int64_t& timestamp);
    void clear(void);

    private:
    std::chrono::high_resolution_clock::time_point init_time_point;
    size_t max_size;
    std::deque< std::pair<int64_t, Struct> > topic_deque;
    int64_t timestamp(void);
};

template<typename Struct>
void update(Topic<Struct>& topic, Struct content, std::mutex& mtx);

template<typename Struct>
bool latest(Topic<Struct>& topic, int64_t& timestamp, Struct& content, std::mutex& mtx);

template<typename Struct>
void recent(Topic<Struct>& topic, std::vector< std::pair<int64_t, Struct> >& topic_vector, int64_t& timestamp, std::mutex& mtx);

template<typename Struct>
void clear(Topic<Struct>& topic, std::mutex& mtx);

template<class Struct>
Topic<Struct>::Topic(std::chrono::high_resolution_clock::time_point init_timepoint, size_t max_size)
{
    this->init_time_point = init_timepoint;
    this->max_size = std::max(max_size, (size_t)1);
    return;
}

template<class Struct>
Topic<Struct>::~Topic()
{
    return;
}

template<class Struct>
void Topic<Struct>::update(const Struct content)
{
    std::pair<int64_t, Struct> topic = std::make_pair(timestamp(), content);
    if( topic_deque.size() >= max_size )
    {
        for(size_t i = 0; i < 1 + topic_deque.size() - max_size; i++)
        {
            topic_deque.pop_front();
        }
    }
    topic_deque.push_back(topic);
    return;
}

template<class Struct>
bool Topic<Struct>::latest(int64_t& timestamp, Struct& content)
{
    bool valid;
    std::pair<int64_t, Struct> topic;
    if( ! topic_deque.empty() )
    {
        topic = topic_deque.back();
        timestamp = topic.first;
        content = topic.second;
        valid = true;
    }
    else{
        valid = false;
    }
    return valid;
}

template<class Struct>
void Topic<Struct>::recent(std::vector< std::pair<int64_t, Struct> >& topic_vector, int64_t& timestamp)
{
    topic_vector.clear();
    std::pair<int64_t, Struct> topic;
    if( ! topic_deque.empty() )
    {
       for(size_t i = 0; i < topic_deque.size(); i++)
        {
            topic = topic_deque[i];
            if( topic.first > timestamp )
            {
                timestamp = topic.first;
                topic_vector.push_back(topic);
            }
        }
    }
    return;
}

template<class Struct>
void Topic<Struct>::clear( void )
{
    topic_deque.clear();
    return;
}

template<class Struct>
int64_t Topic<Struct>::timestamp(void)
{
    std::chrono::duration<double> time_span = std::chrono::high_resolution_clock::now() - init_time_point;
    std::chrono::milliseconds d = std::chrono::duration_cast<std::chrono::milliseconds>(time_span);
    return d.count();
}

template<typename Struct>
void update(Topic<Struct>& topic, Struct content, std::mutex& mtx)
{
    mtx.lock();
    topic.update(content);
    mtx.unlock();
    return;
}

template<typename Struct>
bool latest(Topic<Struct>& topic, int64_t& timestamp, Struct& content, std::mutex& mtx)
{
    bool valid = false;
    mtx.lock();
    valid = topic.latest(timestamp, content);
    mtx.unlock();
    return valid;
}

template<typename Struct>
void recent(Topic<Struct>& topic, std::vector< std::pair<int64_t, Struct> >& topic_vector, int64_t& timestamp, std::mutex& mtx)
{
    topic_vector.clear();
    mtx.lock();
    topic.recent(topic_vector, timestamp);
    mtx.unlock();
    return;
}

template<typename Struct>
void clear(Topic<Struct>& topic, std::mutex& mtx)
{
    mtx.lock();
    topic.clear();
    mtx.unlock();
    return;
}
#endif
