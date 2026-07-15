# v4 TrackingModeFSM Q&Aレポート

**作成日**: 2026-07-15
**対象**: `px_sim_v4`（①〜④実装、挙動5・8・11・12統合完了時点）
**位置づけ**: v4実装完了後に行われたQ&Aセッションの質問と回答を、レビュー・引き継ぎ用にまとめたもの。

---

## Q1. watch modeとdetect modeでどのような動きをしているかまとめてください

| | DETECT_TRACK mode | WATCH mode |
|---|---|---|
| **actuatorへの新規指令** | 出す（`track_target_deg()`が毎フレームPD計算し`data.ctrl`を更新） | 出さない（`track_target_deg()`は早期リターンし、`data.ctrl`は直前の値を保持） |
| **PD状態(`pos_deg`/`prev_err_deg`)** | 毎フレーム更新される | 変化しない（凍結される） |
| **joint(qpos)の物理的な動き** | targetへ向かって動く | 原則静止。ただしWATCH突入直前に送信済みの`ctrl`へ向かう物理的な収束运動が残っていた場合、その分だけは慣性で動き続ける（新規指令ではないので挙動としては正常） |
| **`no_det_frames`カウント** | `on_detection_result(False)`のたびにカウントアップ。`lost_max_frames`到達で自動的にWATCHへ遷移 | 0で固定（カウントしない） |
| **モード遷移のトリガー** | ロストフレーム連続（挙動8）→WATCHへ | 検出成功イベント（`enter_detect_track()`呼び出し）→DETECT_TRACKへ |
| **遷移時に起こる処理** | ― | ―（下記の通り2種類ある） |

**遷移の詳細:**
- **WATCH → DETECT_TRACK** (`enter_detect_track()`): `no_det_frames`を0にリセット、`first_detect_frame`フラグを立てる、PD制御器(`yaw_pd`/`pitch_pd`)を`reset(0.0)`する。次の1フレームだけD項がスキップされる（挙動5）。
- **DETECT_TRACK → WATCH** (`enter_watch()`): `no_det_frames`を0にリセットするのみ。PD制御器の状態(`pos_deg`/`prev_err_deg`)はリセットされず、直前の追従状態をそのまま保持する。

---

## Q2. watch modeはservoの停止のみなのになぜwatch modeを入れる必要がある？

**ご提示の解釈（Detect modeから抜ける→watch modeに入る→detect modeに戻る、という一連の動きを再現するためにwatch modeを設けている。基本的にwatch modeでは何もしない）は正しいです。**

補足すると、「何もしない」ことそのものが、次の2つの意味で意図的な設計です。

1. **意図しない指令の暴走を防ぐため。** ターゲットを見失った状態で仮に`track_target_deg()`を呼び続けてしまうと、誤差計算に使う「真の目標角」が存在しないため、ノイズや古い値を追い続けて予期しない方向へservoが動くリスクがある。WATCHで指令送信自体を止めることで、この暴走を構造的に防いでいる。
2. **DETECT_TRACK復帰時の状態を保証するため。** WATCH中はPD状態(`pos_deg`/`prev_err_deg`)を変更しないため、再度`enter_detect_track()`が呼ばれた瞬間に、確実に「初回DETECTフレーム」という既知の初期状態からD項スキップ付きで再開できる。もしWATCHが存在せず単純に指令を止めるだけなら、この「モードの境界」自体が曖昧になり、挙動5（初回D項スキップ）のトリガー条件が定義できなくなる。

つまりWATCH modeは「何もしない状態」を明示的なstateとして持つことで、①遷移の安全性、②遷移後の初期状態の再現性、の両方を担保する役割を持っています。

---

## Q3. TrackingModeFSMの関数とその役割（添削）

| 関数 | ご提示の解釈 | 判定 | 補足・訂正 |
|---|---|---|---|
| `reset()` | 状態の初期化 | ✅ 概ね正しい | `mode`をWATCHに戻し、`no_det_frames`・`first_detect_frame`を0/Falseにする。`Env.reset()`から呼ばれる |
| `enter_watch()` | 何もしないが`mode_`と`no_det_frames_`の初期化のみする | ✅ 正しい | 補足: `mode`をWATCHにし`no_det_frames`を0にする、の2点のみ。PD状態には一切触れない(Q1参照) |
| `is_watch()`, `is_detect()` | detectとwatch modeかどうかの確認 | ✅ 正しい | `self.mode == Mode.WATCH` / `Mode.DETECT_TRACK`の単純な比較 |
| `enter_detect_track()` | pitch, yaw角の初期化 | 🔶 一部訂正 | **PD制御器のリセット自体はこのメソッドの中では行わない。** `mode`をDETECT_TRACKにし、`no_det_frames`を0、`first_detect_frame`をTrueにするのみ。実際のyaw/pitch PD状態のリセット(`yaw_pd.reset(0.0)`/`pitch_pd.reset(0.0)`)は、Env側の`env.enter_detect_track()`(FSMのメソッドと同名だが別物)が担当する。責務分離のため、FSM自体はactuator/PDに触れない設計にしている |
| `consume_first_detect_frame()` | detect modeでの一回目のframeの使用を明示的に示す | ✅ 正しい | `first_detect_frame`を読み取り、Falseにリセットして返す。「1回だけdt=0を使う」という一回性の消費を表す |
| `on_detection_result()` | わからない | 📝 説明を追加 | 挙動8(ロストフレームによるWATCH復帰)の本体。検出成功/失敗のイベントを毎フレーム受け取り、①検出成功なら`no_det_frames`を0にリセット、②検出失敗なら`no_det_frames`をカウントアップし`lost_max_frames`に到達したら自動的に`enter_watch()`を呼びWATCHへ遷移する。WATCH状態で呼ばれた場合は何もしない(実機がWATCH中このコールバック自体を呼ばないことの再現)。戻り値は「このフレームで自動遷移が発生したか」を示すbool |

**重要な訂正点（Q3-enter_detect_track）:**
`enter_detect_track()`という同名のメソッドが2箇所に存在します。

- `TrackingModeFSM.enter_detect_track()` — FSM内の状態(`mode`/`no_det_frames`/`first_detect_frame`)のみを変更。PD/actuatorには触れない。
- `PxPanTiltEnv.enter_detect_track()` — 上記FSMのメソッドを呼んだ**後**に、`yaw_pd.reset(0.0)`/`pitch_pd.reset(0.0)`を呼び、PD制御器もあわせてリセットする。呼び出し順序をこのメソッド1箇所に固定することで、04_future_problem_prediction.mdの2-A（mode遷移時のリセット順序の取り違え）を防いでいる。

ご質問への回答は後者（Env側）の挙動を指していたと思われますが、実装上は前者（FSM）と後者（Env）が責務分離されている点が実装のポイントです。

---

## Q4. Test5でのtest内容の添削

**test5はa〜iの9個で構成されています**（ご質問の「a〜h」という範囲指定は実際には9個中8個に相当し、test5-iが1個抜けている形になります。以下、全9個を対象に添削します）。

| test | ご提示の解釈 | 判定 |
|---|---|---|
| test5-a | （個別記載なし） | FSM単体（Env非依存）の状態遷移テスト。WATCH初期状態、DETECT_TRACK遷移、`consume_first_detect_frame()`の一回性、WATCH復帰時の`no_det_frames`リセットを確認 |
| test5-b | detect modeに入る時必要なparameterがリセットされているか | ✅ 正しい | `env.enter_detect_track()`呼び出し時、FSM遷移とPDリセット(`prev_err_deg`/`pos_deg`)が揃って発生することを確認 |
| ~~test5-c~~ | 「設定patternごとにwatch modeへ正常に戻るか」（原文はtest5-aの説明として記載） | 🔶 test5-cの説明として該当 | `enter_watch()`呼び出し時、FSM状態のみ変化しPD状態は保持されること(=Q1の「WATCH復帰時はPDをリセットしない」ことの検証)。「正常に戻るか」というより「戻った際に何が変化し何が変化しないか」の確認 |
| test5-d | （a〜hの範囲に含まれる想定） | ②(挙動5)の検証。`track_target_deg()`内部でdtが自動的に0へ上書きされ、一回性消費が機能することを確認 |
| test5-e | （同上） | ③(挙動8)の検証。`lost_max_frames`回連続の検出失敗で正確にその回にWATCHへ自動遷移することを確認 |
| test5-f | （同上） | ③(挙動8)の検証。連続失敗の途中で検出成功を挟むと`no_det_frames`がリセットされ、遷移しないことを確認 |
| test5-g | （同上） | ③(挙動8)の検証。WATCH中に`on_detection_result()`を呼んでも無害であることを確認 |
| test5-h | （同上） | ④(挙動12)の検証。WATCH中は`track_target_deg()`がactuator/PD状態を一切変更せず早期リターンすることを確認 |
| **test5-i** | これがtest5のcore部分で、detect mode ⇒ watch mode ⇒ detect modeの一連のpatternのなかで正常に動くかの確認 | ✅ 正しい | ご認識の通り。実際に`env.step()`でjointを物理的に動かし、①detectで移動→②lost framesでwatch自動遷移→③watch中は静止(0.01度未満の許容誤差)→④detect復帰で新targetへ再び動き出す、の4フェーズを1つのシナリオとして検証。他のtest(a〜h)が個々の要素の「設定した内容が想定通りに動くか」を確認する単体テストであるのに対し、test5-iはそれらを統合した結合テストという位置づけで、ご認識通りcore部分にあたる |

**添削まとめ**: 全体としての捉え方（test5-a〜hが要素ごとの単体確認、test5-iが統合的なcoreシナリオ）は正しい構造理解です。ただし個別の内容説明はいくつかのtestで入れ替わっていたため、上表で対応関係を訂正しました。

---

## Q5. 挙動5・8・11・12はtest5のa〜iのうちどれに該当するか

| 挙動 | 該当するtest |
|---|---|
| **挙動11**（watch/detect 2-mode遷移） | test5-a, test5-b, test5-c |
| **挙動5**（初回DETECTフレームのD項スキップ） | test5-d |
| **挙動8**（ロストフレームによるWATCH復帰） | test5-e, test5-f, test5-g |
| **挙動12**（WATCHモードのfps間引き→「WATCH中は新規指令を出さない」という帰結） | test5-h |
| **（統合シナリオ）** | test5-i（挙動5・8・11・12すべてを1つの一連の流れとして検証） |

補足：test5-iは特定の挙動1つに対応するテストではなく、①〜④で実装した4つの挙動が組み合わさって実機通りの一連の状態遷移パターン（動く→止まる→また動く）を再現できているかを確認する統合テストです。そのため上表では独立した行として扱っています。
