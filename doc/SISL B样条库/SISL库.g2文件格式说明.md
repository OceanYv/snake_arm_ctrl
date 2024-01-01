# SISL库.g2文件格式说明

- 参考自SISL库中 streaming/src/GoReadWrite.cpp的 writeGoSurface、writeGoCurve函数

- 曲线

```bash
100 1 0 0                 # 这几个都是常数 
3 0                       # 维数、类型（0： 多项式B样条，1：有理B样条）    
4 4                       # Number of vertices、 Order of curve
0 0 0 0 1 1 1 1           # knotvector，个数为number of vertices + Order of curve
0 0 0 1 2 0 2 3 0 3 0 0   # 控制点（array of vertices），个数为vertices，一个点包含的数据个数为维数（如果是有理B样条，维度+1）
```

- 曲面

```bash
200 1 0 0       # 这几个都是常数                                   go_stream << SURFACE_INSTANCE_TYPE << ' ' << MAJOR_VERSION << ' '   << MINOR_VERSION << " 0\n";
3 0             # 维数、类型（0： 多项式B样条，1：有理B样条）          go_stream << dim << ' ' << rational << '\n';
5 2             # u方向上的Number of vertices、 Order of surface  os << n << ' ' << k << '\n';
0 0 1 2 3 4 4   # u方向上的knotvector                             for (int i = 0; i < n + k; ++i)  os << knots[i] << ' ';    os << '\n';
5 2             
0 0 1 2 3 4 4 
0 0 0 1 0 0 2 0 0 3 0 0 4 0 0 0 1 0 1 1 0 2 1 0 3 1 0 4 1 0 0 2 0 1 2 0 2 2 1 3 2 0 4 2 0 0 3 0 1 3 0 2 3 0 3 3 0 4 3 0 0 4 0 1 4 0 2 4 0 3 4 0 4 4 0  # 控制点（array of vertices），个数为vertices1*vertices2，一个点包含的数据个数为维数（如果是有理B样条，维度+1）
```

- 一个g2文件中支持写入多个曲线或曲面
