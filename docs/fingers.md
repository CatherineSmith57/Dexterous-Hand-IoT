一、ORCA Hand v2 右手 — 17个关节详解
关节名	中文	电机ID	ROM范围(度)	功能说明
wrist	手腕	1	-65 ~ +35	手腕屈/伸（flex/extend）
thumb_cmc	拇指根部	17	-45 ~ +33	拇指对掌/外展（最重要的抓取关节）
thumb_abd	拇指外展	14	-18 ~ +55	拇指张开/并拢
thumb_mcp	拇指掌指	15	-60 ~ +90	拇指弯曲/伸直
thumb_dip	拇指远端	16	-55 ~ +107	拇指指尖弯曲
index_abd	食指外展	4	-30 ~ +25	食指左右摆动
index_mcp	食指掌指	3	-60 ~ +100	食指根部弯曲（最主要的弯曲关节）
index_pip	食指近端	2	-15 ~ +107	食指中间关节弯曲
middle_abd	中指外展	5	-27 ~ +27	中指左右摆动
middle_mcp	中指掌指	10	-60 ~ +100	中指根部弯曲
middle_pip	中指近端	9	-15 ~ +107	中指中间关节弯曲
ring_abd	无名指外展	6	-27 ~ +27	无名指左右摆动
ring_mcp	无名指掌指	7	-60 ~ +100	⚠️ 反向关节，无名指根部弯曲
ring_pip	无名指近端	8	-15 ~ +107	无名指中间关节弯曲
pinky_abd	小指外展	13	-30 ~ +30	⚠️ 反向关节，小指摆动
pinky_mcp	小指掌指	12	-60 ~ +100	小指根部弯曲
pinky_pip	小指近端	11	-15 ~ +107	⚠️ 反向关节，小指中间弯曲

5根手指：
├── 拇指(Thumb) — 4个关节：cmc + abd + mcp + dip
├── 食指(Index) — 3个关节：abd + mcp + pip（无dip）
├── 中指(Middle) — 3个关节：abd + mcp + pip
├── 无名指(Ring) — 3个关节：abd + mcp + pip
└── 小指(Pinky) — 3个关节：abd + mcp + pip