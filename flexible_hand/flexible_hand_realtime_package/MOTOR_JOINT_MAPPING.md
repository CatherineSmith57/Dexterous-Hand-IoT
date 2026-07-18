# 电机 ID 与关节名称对应表

| Motor ID | Joint |
|---:|---|
| 1 | wrist |
| 2 | index_pip |
| 3 | index_mcp |
| 4 | index_abd |
| 5 | middle_abd |
| 6 | ring_abd |
| 7 | ring_mcp |
| 8 | ring_pip |
| 9 | middle_pip |
| 10 | middle_mcp |
| 11 | pinky_pip |
| 12 | pinky_mcp |
| 13 | pinky_abd |
| 14 | thumb_abd |
| 15 | thumb_mcp |
| 16 | thumb_dip |
| 17 | thumb_cmc |

`runtime_config.py` 现在同时支持：

```yaml
motor_limits:
  1: [1742, 3204]
  2: [467, 2423]
```

和：

```yaml
joints:
  wrist:
    raw_min: 1742
    raw_max: 3204
```
