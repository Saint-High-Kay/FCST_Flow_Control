# COMET CDC Gas Replacement Control System

## 1. 概要

本ソフトウェアは、COMET Phase-I実験のCentral Drift Chamber（CDC）で使用するガスの置換・循環運転を支援するPythonプログラムである。FUJIKIN FCSTシリーズのマスフローコントローラー（MFC）3台をRS-485経由で制御し、Heとi-C4H10（イソブタン）の混合比を90:10に保ちながら、CDC内圧に応じて供給流量を調整することを目的とする。

CDC内圧は、既存のSlowMonitorシステムが出力するGL840ログファイルを参照する。本ソフトウェアはGL840と直接通信せず、ログファイルの最新データを読み取る。

本ソフトウェアの通常起動モードはDRY RUNであり、MFCへの流量設定書き込みは行わない。実機への自動書き込みは、起動オプション`--apply`とオペレーターによる明示的な確認を必要とする。

> **注意:** 本READMEおよび各コードの設定値は、現在の開発段階の構成を示す。実際のガスを流す前に、圧力制御条件、流量範囲、配管・排気系、緊急排気バブラーの状態を確認し、DRY RUNと個別のMFC動作試験を行うこと。圧力制御の変更率は初期設定値であり、実機の圧力応答を用いた調整が必要である。

## 2. ハードウェア構成

本ソフトウェアが管理するMFCは以下の3台である。

| MFC名 | 用途 | シリアル番号 | MAC ID | フルスケール | 最小正流量設定 |
|---|---|---|---:|---:|---:|
| Cir01 | He供給 | `AT18ZQW-00` | `0x21` | 5000 sccm（5 SLM） | 100 sccm（2% F.S.） |
| Cir02 | i-C4H10供給 | `AT1MSM9-00` | `0x22` | 400 sccm | 8 sccm（2% F.S.） |
| Exh01 | CDC排気（He:i-C4H10 = 90:10の混合ガス） | `AT1MSM6-00` | `0x24` | 1000 sccm（1 SLM） | 20 sccm（2% F.S.、暫定値） |

3台とも校正ガスはHe:i-C4H10 = 90:10の混合ガスである。Cir01にはHe、Cir02にはi-C4H10、Exh01には混合ガスが流れる。

### 2.1 ガス組成と流量補正係数

本システムの目標混合比は次のとおりである。

- He：90%
- i-C4H10：10%

FUJIKINから提示された流量補正係数（CF）は以下を使用する。

| ガス | CF |
|---|---:|
| He | 1.428 |
| i-C4H10 | 0.210 |
| He:i-C4H10 = 90:10混合ガス | 0.946 |

MFCは混合ガスで校正されている一方、供給側のCir01とCir02には純ガスが流れる。そのため、`gas/gas_replacement.py`では校正ガスと実際に流すガスのCF比を用いてMFC設定値を計算する。

### 2.2 MFCの共通動作流量範囲

3台すべてのMFCが正の流量指令で同時に2～100% F.S.の範囲に入るようにすると、現在のCF・MFCプロファイルにおける混合ガス合計流量の共通範囲は、約167.72～887.95 sccmとなる。

0 sccmは通常の正流量範囲とは別の専用停止指令として扱う。したがって、現在の設定では0 sccm、または約167.72～887.95 sccmを使用できる。0より大きく167.72 sccm未満の流量は、3台すべての最小流量条件を満たさないため使用できない。

Exh01の最小流量20 sccmは、`mfc_profiles.py`で実機確認前の暫定値として定義している。実機仕様・試験結果に基づき確認すること。

## 3. ディレクトリ構成

```text
fcst_gas_replacement/
│
├── README.md
├── fcst_gas_replacement_controller.py
│
├── config/
│   └── mfc_profiles.py
│
├── mfc/
│   ├── fcst_driver.py
│   └── mfc_manager.py
│
├── gas/
│   └── gas_replacement.py
│
├── control/
│   ├── flow_controller.py
│   └── pressure_controller.py
│
├── logging/
│   └── data_logger.py
│
└── data/
    └── mfc_flow.dat  （実行時に自動生成）
```

`logging/data_logger.py`はMFC流量を記録するプログラム本体である。`data/mfc_flow.dat`は実行時に生成される記録ファイルであり、プログラム本体とは別のものである。メインコントローラーは`data/data_logger.py`または`logging/data_logger.py`のいずれかを読み込む。両方に同名ファイルを置いた場合は`data/`側が優先される。

各ディレクトリの役割は以下のとおりである。

| ディレクトリ | 役割 |
|---|---|
| `config/` | MFCごとの個体情報・流量仕様の定義 |
| `mfc/` | FCST MFCとの通信および状態・設定の管理 |
| `gas/` | 混合比とCFに基づくMFC設定流量の計算 |
| `control/` | 圧力フィードバック判断と複数MFCの協調制御 |
| `logging/` | MFC流量データの記録処理 |
| `data/` | 実行時に生成される測定データの保存先 |

## 4. ソフトウェア全体の処理フロー

メインコントローラーは、次の順番で各モジュールを呼び出す。

1. RS-485通信を初期化し、`read_all_status()`で3台のMFCの個体識別と状態を確認する。
2. GL840ログファイルから最新のCDC内圧とバッファータンク圧力を読み取る。
3. CDC内圧に基づいて、圧力コントローラーが次の混合ガス合計流量を計算する。
4. ガス置換モジュールが、He・i-C4H10・排気の各MFC設定流量を計算する。
5. フローコントローラーが3台すべての設定値を事前確認し、流量の増減に応じた指令順序を決定する。
6. DRY RUNでは指令計画を表示する。`--apply`を指定し、オペレーターが確認した場合のみMFCへ書き込む。
7. MFCのSetpointおよびIndicated Flowを読み取り、DATファイルへ記録する。

MFC通信は1つのRS-485接続を共有し、逐次実行する。複数の処理が同時にシリアル通信を行うことはない。

## 5. 各プログラムの詳細

### 5.1 `config/mfc_profiles.py` — MFCプロファイル

**役割**

制御対象MFCの個体情報、流量範囲、校正ガス情報を一元管理する。

**主な定義**

- `MFCProfile`：MFC 1台分の設定情報を保持するデータクラス。
- `MFC_PROFILES`：Cir01、Cir02、Exh01のプロファイル一覧。
- `get_profile()`：MFC名または別名からプロファイルを取得する。
- `validate_profile_table()`：MAC IDやシリアル番号の重複などを検証する。

**プロファイルの主な項目**

- `name`：MFC名。
- `role`：用途。
- `serial_number`：シリアル番号。
- `mac_id`：RS-485通信で使用するMAC ID。
- `full_scale_sccm`：フルスケール流量。
- `min_flow_sccm`、`max_flow_sccm`：正流量の設定範囲。
- `calibration_gas`、`calibration_cf`：校正ガスとそのCF。
- `native_unit`：MFCの仕様上の流量単位。

アプリケーション内で扱う流量はsccmに統一する。MFCを交換した場合や仕様を変更した場合は、このプロファイルを更新する。また、`gas/gas_replacement.py`および`control/pressure_controller.py`にも関連する流量仕様・CF値が定義されているため、それらとの整合性も確認すること。

### 5.2 `mfc/fcst_driver.py` — FCST通信ドライバー

**役割**

FUJIKIN FCSTシリーズMFCとのRS-485通信を担当する低レベルドライバーである。通信フレームの生成・送受信、応答の検証、レジスタ値の読み書きを行う。

**通信仕様**

- 通信方式：RS-485
- ボーレート：38400 bps
- データ長：8 bit
- パリティ：なし
- ストップビット：1 bit
- READコマンド：`0x80`
- WRITEコマンド：`0x81`
- STX：`0x02`
- WRITE応答：ACK 2 byte（`0x06 0x06`）

**主な機能**

- USB-RS485変換器（B&B ULINX USOPTL4）のシリアルポートを開く。
- MAC IDを指定してMFCのレジスタを読み出す。
- 指定レジスタに値を書き込む。
- 応答フレームのレジスタID、データ長、PAD、チェックサムを検証する。
- タイムアウト、NAK、不正フレームなどを例外として通知する。

**主な読み出しレジスタ**

| 項目 | Class | Instance | Attribute | 内容 |
|---|---:|---:|---:|---|
| Serial Number | `0x64` | `0x01` | `0x07` | シリアル番号 |
| Configured MAC ID | `0x03` | `0x01` | `0x01` | 設定MAC ID |
| Control Mode | `0x69` | `0x01` | `0x03` | 現在の制御モード |
| Default Control Mode | `0x69` | `0x01` | `0x04` | デフォルト制御モード |
| Freeze Follow | `0x69` | `0x01` | `0x05` | Setpoint反映状態 |
| Setpoint | `0x69` | `0x01` | `0xA4` | 設定流量レジスタ |
| Filtered Setpoint | `0x6A` | `0x01` | `0xA6` | フィルター後設定流量 |
| Indicated Flow | `0x6A` | `0x01` | `0xA9` | MFC指示流量レジスタ |

このドライバーはMFC個体ごとの流量範囲やCF補正を判断しない。それらは上位のプロファイル・管理・ガス計算モジュールが担当する。モジュールの読み込み時に実機通信やMFC設定変更は行わない。

### 5.3 `mfc/mfc_manager.py` — MFC状態・設定管理

**役割**

低レベル通信ドライバーを利用し、MFCの個体識別、状態取得、流量設定値の検証、設定変更、書き込み後の確認を行う。

**主なクラス・機能**

- `MFCStatus`：MFCの識別情報、制御モード、Freeze Follow、Setpoint、Indicated Flowなどを保持する。
- `SetpointPlan`：流量設定前の読み取り専用計画を保持する。
- `MFCManager.read_status()`：指定したMFCの状態を読み出す。
- `MFCManager.read_all_status()`：有効な全MFCの状態を読み出す。
- `MFCManager.validate_flow()`：MFCごとの正流量範囲を検証する。
- `MFCManager.plan_flow()`：個体識別・状態・目標流量を確認し、書き込み計画を作成する。
- `MFCManager.set_flow_sccm()`：明示的な許可がある場合に流量を書き込み、読み戻し確認を行う。

**Setpointの換算**

FCSTのSetpoint・Indicated Flowレジスタは16 bitの値であり、`0x4000`をゼロ、`0xC000`をフルスケールとして線形換算する。換算には各MFCの`full_scale_sccm`を用いる。ガス固有のCF補正はこの管理モジュールでは行わず、ガス計算モジュールで扱う。

**制御モードとFreeze Follow**

流量を書き込む際、現在のControl ModeがAnalogであればDigitalへ切り替え、その状態を読み戻して確認する。Freeze Followが設定値を適用する状態でなければ、Setpointをバッファーに書き込み、Freeze Followを適用状態にしてから最終確認を行う。Default Control Modeは変更しない。

**書き込み条件**

`set_flow_sccm()`は、`apply=True`かつ`confirmed=True`の両方が指定された場合に限りWRITEを行う。通常の事前確認では書き込みを行わない。

### 5.4 `gas/gas_replacement.py` — ガス置換流量計算

**役割**

目標とする90:10混合ガスの合計流量から、Cir01、Cir02、Exh01に設定する各MFC流量を計算する。RS-485通信やMFCへの書き込みは行わない。

**流量の分配**

合計混合ガス流量を\(Q_{\mathrm{mix}}\)とすると、実ガス流量の目標値は次のとおりである。

\[
Q_{\mathrm{He}}=0.90Q_{\mathrm{mix}}
\]

\[
Q_{\mathrm{iC_4H_{10}}}=0.10Q_{\mathrm{mix}}
\]

\[
Q_{\mathrm{Exh}}=Q_{\mathrm{mix}}
\]

**CF補正**

混合ガスで校正されたMFCに別のガスを流す場合、設定流量は次式で求める。

\[
Q_{\mathrm{set}}=Q_{\mathrm{actual}}\times
\frac{CF_{\mathrm{calibration}}}{CF_{\mathrm{flowing}}}
\]

Cir01はHeのCF、Cir02はi-C4H10のCF、Exh01は混合ガスのCFを用いる。

**計算例：合計200 sccmの場合**

| MFC | 実ガス流量の目標 | MFCに設定する流量（CF補正後） |
|---|---:|---:|
| Cir01 | He 180 sccm | 約119.244 sccm |
| Cir02 | i-C4H10 20 sccm | 約90.095 sccm |
| Exh01 | 混合ガス 200 sccm | 200.000 sccm |

ここでいうMFC設定流量は、MFCが混合ガス校正スケールで受け取る設定値である。MFCのSetpointレジスタへの量子化は`mfc_manager.py`が担当する。

**流量範囲の検証**

3台それぞれの最小流量・フルスケールを確認し、3台が同時に動作できる合計流量範囲を求める。許容範囲外の正流量はエラーとし、勝手に上下限へ丸めない。0 sccmは3台に対する専用停止計画として扱う。

### 5.5 `control/pressure_controller.py` — CDC内圧フィードバック判断

**役割**

既存のSlowMonitorシステムが保存するGL840ログファイルから最新のCDC内圧を読み取り、次の混合ガス合計流量を計算する。GL840との直接通信およびMFCへの書き込みは行わない。

**参照ファイル**

デフォルトでは次のファイルを読み込む。

```text
/home/cdc/SlowMonitor/dat/gl840.dat
```

使用する列（0始まり）は以下のとおりである。

| 列番号 | 内容 | 単位 |
|---:|---|---|
| 0 | Unix時刻 | s |
| 7 | CDC内圧（`dif_p_in`） | Pa |
| 8 | バッファータンク差圧（`dif_p_buf`） | Pa |

このファイル自体を生成するGL840データ取得プログラムは既存SlowMonitor側の機能であり、本リポジトリには含めない。

**データの有効性確認**

- 最後まで書き込まれた完全な行を読み取る。
- タイムスタンプ、CDC内圧、タンク圧力が数値として有効か確認する。
- データの経過時間が120秒を超えた場合は古いデータとしてエラーにする。
- タイムスタンプが現在時刻より5秒を超えて未来の場合もエラーにする。

**圧力制御条件（初期設定）**

目標CDC内圧は100 Paとする。現在の指令合計流量に対し、CDC内圧に応じて次の割合で変更する。

| CDC内圧 | 合計流量の変更 | 判断 |
|---|---:|---|
| 120 Pa超 | −10% | 大きく減少 |
| 100 Pa超～120 Pa以下 | −2% | 小さく減少 |
| 50 Pa以上～100 Pa以下 | 変更なし | 維持 |
| 20 Pa以上～50 Pa未満 | +5% | 増加 |
| 0 Pa以上～20 Pa未満 | +10% | 大きく増加 |
| 0 Pa未満 | +20% | より大きく増加 |

現在流量が0 sccmの場合は、圧力値に関係なく停止状態を維持し、自動的にガス供給を再開しない。

**流量範囲の制限**

正の合計流量は、現在の3台のMFCが同時に動作できる共通範囲（約167.72～887.95 sccm）に制限する。下限に達しても、圧力制御の判断だけで自動停止指令を出さない。タンク圧力は読み取って表示するが、現在の制御判断に使用する圧力はCDC内圧である。

上記の変更率は旧`feedback_control.py`の圧力帯の考え方を参考に設定した初期値であり、新しいMFC・配管系での圧力応答を評価して調整する必要がある。

### 5.6 `control/flow_controller.py` — 3台のMFC協調制御

**役割**

ガス置換モジュールが計算した3台分の設定流量を、既存の`MFCManager`を通じて順番にMFCへ適用する。書き込み前に3台すべてを事前確認し、どれか1台でも不整合があれば書き込みを開始しない。

**主な処理**

1. 現在の指令合計流量と目標合計流量を比較する。
2. `gas_replacement.py`から3台分の設定値を計算する。
3. `gas_replacement.py`と`mfc_profiles.py`のMAC ID、シリアル番号、フルスケール、最小流量の整合性を確認する。
4. `MFCManager.validate_flow()`および`plan_flow()`で各MFCの設定可能範囲・状態・個体識別を確認する。
5. 変化方向に応じて書き込み順序を決める。
6. 明示的な確認を受けた場合のみ、順番にMFCへ書き込む。

**MFCへの指令順序**

| 操作 | 指令順序 | 考え方 |
|---|---|---|
| 流量増加 | Exh01 → Cir01 → Cir02 | 先に排気側を設定し、その後に供給を増やす |
| 流量減少 | Cir01 → Cir02 → Exh01 | 先に供給を減らし、その後に排気を減らす |
| 全停止 | Cir01 → Cir02 → Exh01 | 供給を先に停止する |

現在と目標の合計流量の差が0.01 sccm以内であれば、通常は変更不要と判断する。

**エラー時の処理**

書き込み中にエラーが発生した場合、その時点で後続MFCへの書き込みを中止する。既に書き込みが完了したMFC、失敗したMFC、未処理のMFCを報告する。MFC間の設定変更は原子的ではないため、自動ロールバックは行わない。エラー後は3台すべての状態とガス経路を確認してから再開する。

### 5.7 `logging/data_logger.py` — MFC流量ログ

**役割**

3台のMFCからSetpointとIndicated Flowを読み出し、時系列データとしてDATファイルに記録する。MFCへのWRITEは行わない。

**記録項目**

| 列 | 項目 | 内容 |
|---:|---|---|
| 1 | `UnixTime_s` | Unix時刻 |
| 2 | `Cir01_Setpoint_sccm` | Cir01の設定流量 |
| 3 | `Cir01_IndicatedFlow_sccm` | Cir01のMFC指示流量 |
| 4 | `Cir02_Setpoint_sccm` | Cir02の設定流量 |
| 5 | `Cir02_IndicatedFlow_sccm` | Cir02のMFC指示流量 |
| 6 | `Exh01_Setpoint_sccm` | Exh01の設定流量 |
| 7 | `Exh01_IndicatedFlow_sccm` | Exh01のMFC指示流量 |

値は各MFCの16 bitレジスタ値をフルスケールに基づいて線形換算したsccm相当値であり、CF補正は行わない。Indicated FlowはMFC内部の指示値であり、独立した流量計による実測値ではない。

**記録形式と周期**

- 区切り文字：空白
- ファイル形式：DAT（テキスト）
- デフォルト保存先：`data/mfc_flow.dat`
- デフォルト記録周期：1秒
- 記録方式：既存ファイルへの追記

各周期で3台のSetpointとIndicated Flowを読み出す。あるレジスタの読み取りに失敗した場合は該当項目を`nan`として記録し、ほかの項目の取得は続ける。通信時間が1秒を超えた場合は警告を表示する。メインコントローラーと単独実行のどちらでも使用できる。

### 5.8 `fcst_gas_replacement_controller.py` — メインコントローラー

**役割**

圧力フィードバック、ガス流量計算、3台のMFC制御、流量ログ記録を統合する最上位プログラムである。

**起動時の処理**

1. RS-485通信を開始する。
2. `MFCManager.read_all_status()`で3台のシリアル番号・MAC ID・制御状態・流量関連レジスタを読み出す。
3. 起動時に指定された現在の指令合計流量と、DigitalモードのMFCのSetpointが整合するか確認する。
4. 圧力コントローラー、フローコントローラー、データロガーを初期化する。
5. DRY RUNまたは`--apply`による実機制御を開始する。

AnalogモードのMFCでは、Digital Setpointレジスタから現在のアナログ指令値を検証できないため、その点を警告として表示する。

**実行周期**

- 圧力フィードバック：デフォルト5秒。
- MFC流量ログ：デフォルト1秒（ベストエフォート）。

MFCのログ取得とフィードバック制御は、同じRS-485接続を使って逐次実行する。通信処理に時間がかかった場合、記録周期・制御周期は遅れることがある。1秒周期は保証されたリアルタイム周期ではない。

**現在流量の指定**

`--current-flow-sccm`には、起動時点で実際に指令している混合ガス合計流量を指定する。MFCの瞬間的なIndicated Flowの合計ではない。この値は増減方向の判定および起動時のSetpoint整合確認に使われるため、実際の設定状態と一致させること。

現在流量を0 sccmとして起動した場合、プログラムは停止状態を維持し、自動的にガス供給を開始しない。初期流量は別の手順で設定してから、その合計流量を指定して本コントローラーを起動する。

**異常時の動作**

GL840データが古い・不正である場合、MFCの事前確認に失敗した場合、またはMFC書き込みでエラーが発生した場合は、制御ループを終了する。異常時に自動停止や自動ロールバックは行わない。終了後、3台のMFC状態とガス経路を確認すること。

## 6. 実行環境と準備

### 6.1 想定環境

- Linux PC
- Python 3.10以降を想定（`dataclass(slots=True)`などを使用）
- B&B ULINX USOPTL4 USB-RS485変換器
- FUJIKIN FCST MFC 3台
- 既存SlowMonitorが出力する`gl840.dat`
- Pythonパッケージ：`pyserial`

`pyserial`が未導入の場合は、利用するPython環境にインストールする。

```bash
python3 -m pip install pyserial
```

### 6.2 作業ディレクトリ

以下のようにプロジェクトのルートディレクトリへ移動して実行する。各モジュールはプロジェクト内のパッケージパスを使っているため、原則としてルートディレクトリから起動する。

```bash
cd /path/to/fcst_gas_replacement
```

### 6.3 RS-485ポート

デフォルトでは`/dev/serial/by-id/`以下から`USOPTL4`を含むポートを1つ検出する。自動検出できない場合、または複数候補がある場合は`--port`でポートを明示する。

```bash
ls -l /dev/serial/by-id/
```

例：

```text
/dev/ttyUSB1
```

この場合は起動時に`--port /dev/ttyUSB1`を指定する。

## 7. 実行方法

### 7.1 MFC状態の読み出し

流量制御を始める前に、3台の個体識別・制御状態を読み出して確認する。読み出し専用の`MFCManager.read_all_status()`を使用し、Setpointなどを書き換えない。

### 7.2 ガス流量計算のみ

RS-485通信を行わず、200 sccmの混合ガス流量に必要な3台の設定値を確認する。

```bash
python3 gas/gas_replacement.py --total-flow-sccm 200
```

### 7.3 圧力制御判断のみ

GL840ログの最新値を読み、現在の指令合計流量を200 sccmとした場合の次の流量目標を計算する。このモジュール単体ではMFC通信・書き込みを行わない。

```bash
python3 control/pressure_controller.py --current-flow-sccm 200
```

GL840ファイルの場所を変更する場合：

```bash
python3 control/pressure_controller.py \
  --current-flow-sccm 200 \
  --data-file /path/to/gl840.dat
```

### 7.4 MFC流量ログのみを実行する場合

`data_logger.py`を単独で起動すると、MFCの個体識別を確認した後、SetpointとIndicated FlowをDATファイルへ記録する。MFCへのWRITEは行わない。

`data_logger.py`を`logging/`に配置している場合：

```bash
python3 logging/data_logger.py --interval 1 --samples 10
```

連続記録する場合は`--samples 10`を省略し、終了時に`Ctrl+C`を押す。保存先を変更する場合は`--output`を指定する。

### 7.5 メインコントローラーのDRY RUN

まずは1回だけログを記録し、圧力制御判断とMFC設定計画を表示する。

```bash
python3 fcst_gas_replacement_controller.py \
  --current-flow-sccm 200 \
  --once
```

継続してDRY RUNを行う場合：

```bash
python3 fcst_gas_replacement_controller.py \
  --current-flow-sccm 200
```

制御サイクル数を指定する場合：

```bash
python3 fcst_gas_replacement_controller.py \
  --current-flow-sccm 200 \
  --max-cycles 10
```

ポートを明示する場合：

```bash
python3 fcst_gas_replacement_controller.py \
  --current-flow-sccm 200 \
  --port /dev/ttyUSB1 \
  --once
```

### 7.6 実機への自動書き込み

`--apply`を指定すると、起動時に警告が表示され、`ENABLE FEEDBACK`と入力した場合のみ自動MFC書き込みが有効になる。

```bash
python3 fcst_gas_replacement_controller.py \
  --current-flow-sccm 200 \
  --apply
```

**実機での圧力・流量応答、配管・排気経路、緊急排気バブラー、初期流量、MFCの設定状態を確認するまでは、`--apply`を使用しないこと。** 初期流量の指定が実際の指令値と一致しない場合、流量変更順序や起動時確認が正しく機能しない可能性がある。

連続運転を終了するには`Ctrl+C`を押す。複数MFCへの書き込み途中に中断した場合、3台の設定が一部だけ更新されている可能性がある。再起動前に全MFCの状態を読み直すこと。

## 8. データファイル

### 8.1 GL840圧力ログ

本ソフトウェアが参照する圧力ログは既存SlowMonitorの出力である。

```text
/home/cdc/SlowMonitor/dat/gl840.dat
```

本リポジトリはこのファイルを読み取るだけで、GL840データの収集・生成は行わない。

### 8.2 MFC流量ログ

メインコントローラーのデフォルト出力先は次のとおりである。

```text
data/mfc_flow.dat
```

先頭行に列名を`#`付きで記録し、以降は1サンプル1行の空白区切りで追記する。記録値はUnix時刻、各MFCのSetpoint、Indicated Flowである。圧力値はこのファイルには記録しない。

## 9. 安全上の注意・既知の制限

1. **DRY RUNが標準である。** `--apply`を指定しない限り、メインコントローラーはMFCへのWRITEを行わない。ただし、MFCの状態読み出しのためRS-485通信は行う。
2. **圧力制御の変更率は初期値である。** 旧制御の圧力帯を参考にした設定であり、実機のCDC圧力応答、配管抵抗、排気能力などを確認して調整する必要がある。
3. **GL840データの更新が120秒を超えると制御を中止する。** ただし、ソフトウェア終了時にMFCへ停止指令を自動送信するわけではない。
4. **MFC間の書き込みは原子的ではない。** 1台目・2台目の設定が更新された後に次の通信が失敗する可能性がある。自動ロールバックは行わない。
5. **1秒記録はベストエフォートである。** 3台のSetpointとIndicated Flowを読み取る通信時間、および圧力制御時の通信処理によって周期が遅れることがある。
6. **Indicated FlowはMFC内部の指示値である。** 独立した流量計による実測値ではなく、DATログではCF補正を行わない。
7. **MFCの流量仕様は複数ファイルで管理されている。** `config/mfc_profiles.py`、`gas/gas_replacement.py`、`control/pressure_controller.py`の流量範囲・CFに関する定義を変更する場合は、相互の整合性を確認する。
8. **Exh01の最小流量20 sccmは暫定値である。** 実機で確認した値に基づいて更新すること。
9. **緊急排気バブラー等のハードウェア上の保護機構は本ソフトウェアの代替ではない。** ガス供給・排気系の安全確認を別途行うこと。

## 10. 開発・検証状況

- Cir01、Cir02、Exh01のRS-485マルチドロップ接続で、シリアル番号とMAC IDを読み出し、登録情報との一致を確認した。
- 各モジュールは模擬通信・模擬データによる計算・処理確認を行っている。
- 実機を用いた連続フィードバック運転、圧力制御パラメータの最適化、1秒間隔の継続的な記録性能は未検証である。
- 実機での自動制御は、DRY RUN、個別MFC動作確認、流量計・圧力応答の確認を経てから実施する。

---

## 11. 変更時の確認項目

MFCの交換、流量範囲の変更、CF値の更新、GL840ログ形式の変更、圧力制御パラメータの変更を行った場合は、関連する以下の項目を確認する。

- MFC個体情報：`config/mfc_profiles.py`
- ガス組成・CF・MFC流量範囲：`gas/gas_replacement.py`
- 圧力ログ列・更新期限・制御条件・共通流量範囲：`control/pressure_controller.py`
- RS-485レジスタ・通信仕様：`mfc/fcst_driver.py`
- Setpointの検証・換算：`mfc/mfc_manager.py`
- 起動時・運転周期・ログ出力：`fcst_gas_replacement_controller.py`

変更後は、少なくとも構文確認、計算確認、DRY RUNを行い、実機に適用する前に個別の確認を実施する。
