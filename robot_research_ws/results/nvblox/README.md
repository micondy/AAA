# Nvblox 验收记录

`moving_loop_20260919_2215.json` 是当前有效的移动映射内容验收；
`moving_route_20260919_2215.{json,csv}` 是对应8段闭合路线。该轮为PASS。

`moving_loop_20260919_2212.json` 证明移动时三维输出有效，但对应路线因Isaac定时退出而未
完成；`moving_route_20260919_2212.json` 正确保留为failed，不用于宣称闭环通过。

`stability_10min_20260919_2218.json` 是N1仿真长时正式结果：600.12秒，状态PASS，包含
各数据流频率/最大间断、非空地图规模以及596次GPU资源采样。对应完成路线为
`stability_route_20260919_2218.{json,csv}`。

`stability_route_long_20260919_2218.json` 是扩大路线后因场景空间限制在第3段超时的失败
记录，不属于通过依据。N1仿真已经完成；Nav2 costmap四点A/B初测已通过，真实D435i和
分类障碍/断流专项A/B仍未完成。
