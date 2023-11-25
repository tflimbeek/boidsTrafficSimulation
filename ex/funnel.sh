#!/bin/sh
for separationWeight in $(awk 'BEGIN{for(i = 1.6; i<2.0; i+=0.05){print i}}') ; do
    for proportionalBrakeDist in $(awk 'BEGIN{for(i = 8.0; i< 8.75; i+= 0.05){print i}}') ; do
        for proportionalBrakeForce in $(awk 'BEGIN{for(i = 8.0; i< 9.0; i +=0.05){print i}}') ; do
            for emergencyDistance in $(awk 'BEGIN{for(i = 5.75; i< 6.5; i+= 0.05){print i}}') ; do
                python3.11 main.py 0.1 6.0 3.0 $emergencyDistance $proportionalBrakeForce $proportionalBrakeDist $separationWeight
            done
        done
    done
done

# for sweepVariable in $(awk 'BEGIN{for(i = 5.5; i<=7.502; i+=0.002){print i}}') ; do
#     #                                   de        fp  dp  fs
#     python3.11 main.py 0.1 6.0 3.0 $sweepVariable 5.0 9.5 2.0
# done


# for brakingMultiplier in $(awk 'BEGIN{for(i=5.0;i<10.0;i+=1.0){print i}}') ; do
#     for emergencyDistance in $(awk 'BEGIN{for(i=2.0;i<=7.5;i+=0.25){print i}}') ; do
#         python3.11 main.py 0.1 6.0 3.0 $brakingMultiplier $emergencyDistance
#     done
# done


# for l in $(awk 'BEGIN{for(i=0;i<=100.0;i+=5){print i}}') ; do
#     for ph in $(awk 'BEGIN{for(i=0.0;i<=1.0;i+=0.05){print i}}') ; do
#         python3.11 main.py 0.26 1.5 1 $l $ph
#     done
# done
