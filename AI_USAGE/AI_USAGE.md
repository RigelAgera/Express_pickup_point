# AI_USAGE

```markdown
## 对话 N：<一句话主题>
**场景**：你为什么去问 AI（解决什么问题 / 探讨什么决策？）
**我的提示词（原文摘录）**：
> ……
**AI 回应要点**（无需贴全文，摘录关键段或截图均可）：
> ……
**我的处理**：[接受 | 修改后采纳 | 拒绝 | 部分采纳]
**理由**：……
```

## 文件组织以及大体思路

### 对话1：<关于文件怎么拆分组织>

**场景**：这方面还是不太会，问问ai有什么想法
**提示词**：

>那这个项目挺难的对我来说。我Dijiska都写不顺。必做部分的文件和选做部分能完全分开吗？

**AI回应**：

![1](image.png)

![2](image-1.png)

```txt
g++ -std=c++17 main_required.cpp common/*.cpp tasks/task1.cpp tasks/task2.cpp tasks/task3.cpp -o required
```

```txt
g++ -std=c++17 main_bonus.cpp common/*.cpp tasks/task4.cpp tasks/task5.cpp tasks/task6.cpp -o bonus
```

**处理**：修改后采纳
**理由**：这个分法挺适合的，min_heap和dijiska放一起吧，文件有点多了。公共基础层就先写对象，再写读入，再写算法
