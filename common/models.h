// 小车和包裹两个对象的定义
// 后续可能需要在 `Package` 里加 `delivered_time`（送达时间 `D_i`）等运行时字段，可以到时候再扩展。
#pragma once


struct package
{
    int id, dest;
    //数据格式说明里提到是实数，何意味，暂时用double吧
    double weight, arrive, deadline;
};

struct car
{
    // 单个包裹不会超过capacity
    double speed, car_weight, capacity;

};