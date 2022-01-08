#ifndef MATH_TOOLS
#define MATH_TOOLS

// 返回符号
#define SIGN(X) (X >= 0 ? 1 : -1)

// 四舍五入
#define ROUND(X) int(X + SIGN(X) * 0.5)

// 约束
#define BOUND(X, MIN, MAX) ((X < MIN ? MIN : X) > MAX ? MAX : (X < MIN ? MIN : X))

#endif /* MATH_TOOLS */