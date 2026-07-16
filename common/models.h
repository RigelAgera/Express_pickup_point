#pragma once

struct package
{
    int id, dest;
    //数据格式说明里提到是实数，何意味，暂时用double吧
    double weight, arrive, deadline;
    /* data */
};

struct car
{
    // 单个包裹不会超过capacity
    double speed, car_weight, capacity;

};