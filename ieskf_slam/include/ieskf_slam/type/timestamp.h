#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <iostream>
namespace IESKFSlam{
    class TimeStamp{
        private:
            uint64_t nsec_;
            double sec_;
        public:
            TimeStamp(uint64_t nsec = 0){
                nsec_ = nsec;
                sec_ = static_cast<double>(nsec)/1e9;
            }
            void fromSec(double sec){
                nsec_ = static_cast<uint64_t>(sec*1e9);
                sec_ = sec;
            }
            void fromNsec(uint64_t nsec){
                nsec_ = nsec;
                sec_ = static_cast<double>(nsec)/1e9;
            }
            const uint64_t & nsec() const {return nsec_;}
            const double & sec() const {return sec_;}
    };
}

#endif