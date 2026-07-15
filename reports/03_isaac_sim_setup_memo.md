# Isaac Sim 環境構築メモ

**作成日**: 2026-07-04
**位置づけ**: MuJoCoでの検証が完了した後、Isaac Simに移る際の参考メモ。
今すぐ環境構築するためのものではなく、必要になった時に何が要るかを把握するための備忘。

---

## なぜMuJoCoの後にするか(前提の再確認)

MuJoCoとIsaac Simは物理エンジンの実装が異なり、同じMJCF/URDFでも
挙動が完全には一致しない(非線形性・接触処理の違い)。そのため
「MuJoCoで固めたロジックをIsaac Simにそのまま移植すれば終わり」ではなく、
Isaac Sim上で改めてチューニング・検証が必要になる。PXの2軸規模では
MuJoCoで大半の検証(PD制御・トルク・mode遷移・ノイズ・遅延)が完結するため、
Isaac Simは「MuJoCoで足りなくなった時」に着手するのが合理的。

## Isaac Simが必要になるタイミングの目安

- センサーシミュレーション(カメラ画像そのもの、LiDAR点群)が要る
- 数百〜数千環境の大規模並列RLをGPUで回したい
- 多関節(脚式・アーム)に拡張し、接触が複雑化した
- 求人票のIsaac Sim経験が実際に問われる段階になった

PXが2軸のままなら、上記のどれにも該当しないため後回しでよい。

---

## 必要環境(現行版の目安)

| 項目 | 要件 |
|---|---|
| GPU | RTX対応必須。RT Coreを持たないGPU(A100/H100等)は非対応 |
| GPU世代目安 | 現行ドキュメント上の最低ラインはRTX 4080相当(世代により変動) |
| VRAM | 8GBは不足気味。16GB以上推奨(16MP超のレンダリングをする場合) |
| OS | Linux推奨。Windows10は既にサポート終了 |
| Python | Isaac Sim 4.x系は Python 3.10 |
| ドライバ | Linux: 580.65.06以降推奨(Ubuntu 22.04.5 + kernel 6.8系の場合) |
| ネット接続 | 必須(アセットのオンライン取得、一部拡張機能) |

## インストール方式(複数あり、用途別)

| 方式 | 向いている人 |
|---|---|
| pipパッケージ(Isaac Sim + Isaac Lab) | 初心者・大半のユーザー。最も簡単 |
| バイナリDL + Isaac Labソースから | pip非対応環境(Ubuntu 20.04等) |
| ソースからフルビルド | Isaac Sim自体を改造したい場合のみ |
| Dockerコンテナ | コンテナ環境で使いたい場合 |

自分の用途(PXの延長)なら pipパッケージ一択で十分。

## 現状のマシンでの位置づけ

これまでの検証(MuJoCo CPU)は追加投資ゼロで完結してきた。Isaac Simは
上記の通りRTX GPU(VRAM16GB目安)が要るため、**着手する場合は
クラウドGPU(Colab Pro/AWS等、既存のColab/AWS学習環境を流用)が現実的**。
ローカルGPU購入は「Isaac Simが本当に必要になった」と判断した後で検討すればよい。

## 参考リンク

- 要件ページ: https://docs.isaacsim.omniverse.nvidia.com/ (バージョンごとにrequirements.htmlあり)
- Isaac Labインストール: https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html
