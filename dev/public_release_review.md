# 公開前レビューの観点

このリポジトリを更新して公開する（GitHub へ push する）たびに使う点検の観点をまとめた。学習者向けの手順書ではなく、リポジトリを保守するための文書である。このリポジトリは Public なので、push はそのまま公開になる。形は、ドローンの教材（[ros2Px4DroneSim](https://github.com/SanghunIm1991/ros2Px4DroneSim)）の同名の文書にそろえ、このプロジェクトの内容（参考にした製品の一般化）に合わせた。

- 対象: リポジトリが追跡しているすべてのファイル（`git ls-files`）と、全履歴（すべてのブランチ・コミット）
- 実施の目安: **push の前に毎回**。手順書をいくつか更新したときにも行う
- 観点は3つ: 1節「機密情報の流出と一般化」、2節「権利侵害」、3節「リンク先が意図どおりか」
- 各節は、コマンドで機械的に拾う「機械点検」と、読んで判断する「目視点検」に分けている。機械点検で何も出なくても、目視点検は省かない
- 結果は、末尾の4節「実施記録」に1行ずつ追記する。指摘と対応の判断は [`dev/qa_log.md`](qa_log.md) にも記録する

コマンドは、リポジトリの直下で実行する。どれもファイルを読むだけで、変更や外部への送信はしない（外部への通信を伴うのは、3-2節で外部リンクを確かめるときだけ）。

## 1. 機密情報の流出と一般化

### 1-1. 見る観点

| 観点 | 具体例 | 主な置き場所 |
|---|---|---|
| 認証情報 | API キー、トークン、パスワード、秘密鍵 | 全ファイル・全履歴 |
| コミットの名義 | author・committer の名前とメールアドレスが、GitHub の noreply アドレス（`<数字>+<ユーザー名>@users.noreply.github.com`）になっているか | `git log` のメタデータ |
| 本文の個人情報 | 実名、実メールアドレス、GitHub のアカウント名（`LICENSE`・`README.md`・`CLAUDE.md` の著作権者の表示、リポジトリの URL、QA 表の著作権者の表記を決めた行は、意図して載せているので対象外） | [`dev/qa_log.md`](qa_log.md)、手順書 |
| PC 固有の情報 | ユーザー名を含むパス、ドライブ構成、RAM・ディスク容量・ビルドの時間などの実測値（README の「確認した環境」を作る場合は、そこだけ対象外。ただし項目が増えていないかは見る） | `docs/`・`dev/` 全体 |
| PC の利用状況 | PC の具体的な用途など、個人の生活を推測させる記述、スペックの制約、PC の扱いへの注意の言及 | [`CLAUDE.md`](../CLAUDE.md)、構想の文書、QA 表 |
| 参考にした製品の一般化 | 製品名・メーカー名（README の「参考にしたもの」の断り書きの1か所だけは対象外）、製品固有のパッケージ名・トピック名・パラメータの値・寸法・通信の詳細、作る側の参考資料のパスと性質を推測させる書き方 | 全ファイル・全履歴・コミットのメッセージ |
| 組み合わせ | 1つずつは無害でも、組み合わせると推測できる情報（README の断り書きと、作る側の参考資料についての記述など） | 文書をまたいで |
| 追跡してはいけないもの | 個人メモと図の生成スクリプト（`notes/`）、練習用のワークスペース（`ros2_ws/`・`ws/`）、ビルドの生成物、走行の記録と地図（`rosbag2_*`・`.pgm` 等）、Git 管理外のローカル設定（`.claude/settings.local.json`・`CLAUDE.local.md`）、`.env`・鍵ファイル、WSL のエクスポート | `.gitignore` と `git ls-files` |
| 画像の写り込み | スクリーンショット（Gazebo・RViz2 の画面等）に、画面上の個人情報・ほかのウィンドウ・書類が写っていないか。メタデータ | `docs/img/`（作ったとき） |

### 1-2. 機械点検

コミットの名義を一覧する（全ブランチ・全履歴）。

```bash
git log --all --format='%an <%ae> | %cn <%ce>' | sort | uniq -c
```

**見方**: 各行の左が件数、`|` の左が author、右が committer。どちらも noreply アドレスになっていれば問題ない。

追跡中のファイルと全履歴から、メールアドレスの形の文字列を拾い、想定済みのもの（noreply・ダミーの `example.com`）を除く。

```bash
git grep -nIE '[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}' | grep -vE 'users\.noreply\.github\.com|noreply@anthropic\.com|@example\.(com|org)|git@github\.com'

git log --all -p | grep -oE '[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}' | grep -vE 'users\.noreply\.github\.com|noreply@anthropic\.com|@example\.(com|org)|git@github\.com' | sort | uniq -c
```

認証情報らしい語と、ユーザー名を含むパス・ドライブのパス・一時フォルダのパスを、今の版と全履歴（過去の版で足した行）から拾う。

```bash
git grep -nIiE 'api[_-]?key|secret|token|passw(or)?d|BEGIN [A-Z ]*PRIVATE KEY|ghp_[A-Za-z0-9]|github_pat_|sk-[A-Za-z0-9]{10}'

git log --all -p | grep -E '^\+' | grep -ciE 'api[_-]?key|secret|token|passw(or)?d|BEGIN [A-Z ]*PRIVATE KEY|ghp_|github_pat_|sk-[A-Za-z0-9]{10}'

git grep -nIE '/home/[a-z][a-z0-9_-]*|[A-Za-z]:\\+Users\\+|/mnt/c/Users/|\b[A-Z]:\\[A-Za-z]|/mnt/[a-z]/|/tmp/claude'

git log --all -p | grep -E '^\+' | grep -oE '/home/[a-z][a-z0-9_-]*|\b[A-Z]:\\[A-Za-z0-9_.\\-]+|/mnt/[a-z]/[A-Za-z0-9_./-]*|/tmp/claude[^ ]*' | sort | uniq -c
```

容量の値と、追跡してはいけないもの（`.gitignore` の項目とそろえている。`.gitignore` を変えたら、ここも変える）と、ブランチ・タグ・過去にだけあったファイルを見る。

```bash
git grep -nIE '[0-9]+(\.[0-9]+)? ?(GB|GiB|MB|TB)\b'

git -c core.quotepath=off ls-files | grep -E '^(notes|ws|ros2_ws|build|install|log|\.vscode|\.idea)/|^\.claude/|^CLAUDE\.local\.md$|(^|/)(Thumbs\.db|Desktop\.ini|\.DS_Store)$|\.(env|pem|key|p12|pfx|tar|zip|vhdx|pgm|posegraph|data)$|rosbag2_'

git branch -a; git tag; git stash list

comm -23 <(git -c core.quotepath=off log --all --format= --name-only | sort -u | grep .) <(git -c core.quotepath=off ls-files | sort)
```

**見方**: メールアドレス・追跡してはいけないもの・過去にだけあったファイルのコマンドは、何も表示されなければ問題ない。認証情報とパスのコマンドは、説明文の中の一般的な語（この文書のコマンドそのもの等）にも当たるので、1件ずつ読んで、実際の値や実在のユーザー名が書かれていないかを判断する。GitHub のアカウント名は、アカウント名そのもので `git grep -n` して、出てきた箇所を目視で判断する（1-1節の表の対象外のものは問題ない）。

**参考にした製品の一般化**: 製品固有の語の一覧による点検（追跡するファイル・全履歴の追加行・コミットのメッセージ）を行う。語の一覧の場所と点検のコマンドは、製品名を含むので、Git 管理外の `CLAUDE.local.md` にだけ書いている。README の断り書きの1か所だけが当たれば問題ない。語の検索では、数値（寸法・パラメータの値等）と言い換えは見つからないので、1-3節の目視で確かめる。

### 1-3. 目視点検

- [`CLAUDE.md`](../CLAUDE.md)・構想の文書・[`dev/qa_log.md`](qa_log.md) に新しく足した行に、アカウント名・実メール・PC 固有の情報・PC の利用状況・個人の練習環境の状態（`notes/` に書くべき内容）が入っていないか。
- 作る側の参考資料について、場所・性質・中身を推測させる書き方になっていないか（「作る側の参考資料」と書き、ほかの語を足さない）。README の断り書きと組み合わせても、推測できることが増えないか。
- 手順書・記録に書いた台車の寸法・設定の値が、この教材で決めた値か（参考資料の値を写していないか）。
- 前回の実施以降に足した画像があれば、1枚ずつ開いて写り込みを確かめる。

## 2. 権利侵害

### 2-1. 見る観点

| 観点 | 具体例 | 主な置き場所 |
|---|---|---|
| 公式の文書の抜粋 | 抜粋した箇所に、帰属表示（出典・ライセンス・改変の有無）が残っているか。抜粋が必要な範囲に留まっているか | 環境構築の手順書、各手順書 |
| 各ページの出典の注記 | 公式の文書と同等のコマンド・API の使い方を含むページに、出典（元にしたものとそのライセンス）と「逐語の転載でない」旨の注記があるか | `docs/` の手順書 |
| 模型・設定のファイル | Gazebo・ROS 2 のパッケージに付属する模型・ワールド・設定のファイルを、リポジトリに含めていないか（読者が各自の導入済みのファイルを使う・写す形にする） | `git ls-files` |
| ツールの出力 | 期待する結果に載せたツールの表示・型の定義の表示が、説明に必要な範囲に留まり、出典があるか | 各手順書 |
| 図 | 自作の図か。第三者の図・ロゴ・埋め込み画像が混ざっていないか | `docs/img/` |
| リポジトリのライセンス | `LICENSE` の第三者の一覧が最新か。コピーレフトの依存が加わっていないか | `LICENSE`、README |

### 2-2. 機械点検

```bash
git -c core.quotepath=off ls-files | grep -vE '\.(md|svg|png|jpg)$|^\.git(attributes|ignore)$|^LICENSE'

git grep -nE '<image|href="https?:|base64,' -- '*.svg'

grep -LE '出典|出どころ|CC BY|License|ライセンス' docs/*.md
```

**見方**: 1つ目は、Markdown・画像・`.gitignore`・`.gitattributes`・`LICENSE` 以外の追跡ファイルを出す（出たものは、自作のものか、第三者のものかを確かめる）。2つ目は、SVG の埋め込み画像と外部参照を出す（何も出なければ問題ない）。3つ目は、出典の注記の語が無い手順書を出す（語の揺れがあるので、出た文書は本文を読んで判断する。学習計画のように出典の要らない文書も出る）。

### 2-3. 目視点検

- 前回以降に足した節が、公式の文書や記事の文章をほぼそのまま写していないか。
- 足したサンプルコードが、公式のコードと一致しすぎていないか。
- 2-1節の表の帰属表示・出典の注記が、文書の書き換えで消えていないか。

## 3. リンク先が意図どおりか

### 3-1. 見る観点

| 観点 | 具体例 |
|---|---|
| 文書間のリンク | 相対パスのリンクの先が実在するか |
| 参照の書き方 | 他の文書への参照がリンクになっているか |
| 節の参照 | 本文の「N節」などの参照が、今の見出しと合っているか |
| 外部リンクの実在 | リンク切れ・移転が無いか。親プロジェクト・ドローンの教材の節を参照している箇所は、参照先の節が今もあるか |
| 外部リンクの中身 | 版の固定（Jazzy・Harmonic の文書等）、リンクの文言と中身が合っているか |

### 3-2. 機械点検

`public-release-review` スキルのリンクの確認のスクリプト（`check_links.py`）を、追跡している Markdown のすべてに対して使う。

```bash
python3 ~/.claude/skills/public-release-review/references/check_links.py internal --files $(git ls-files '*.md')

python3 ~/.claude/skills/public-release-review/references/check_links.py domains --files $(git ls-files '*.md')

python3 ~/.claude/skills/public-release-review/references/check_links.py external --files $(git ls-files '*.md')
```

**見方**: `internal` は、`broken: 0` だけが出れば問題ない。`domains` の一覧を見て、見慣れないドメインが無いかを確かめてから、`external` を実行する（外部への通信を伴うので、実行の前に承認を得る）。`external` は、失敗した URL だけを出す。ボット対策で失敗するサイト（`creativecommons.org`・docs.ros.org 等）は、正式なアドレスか、原稿のリポジトリの同じパスで確かめる。

### 3-3. 目視点検

- 節を足した・繰り下げた文書と、それを参照している文書の「N節」を読み直す。
- 前回以降に足した外部リンクを開いて、文言・版・内容が本文と合っているかを確かめる。

## 4. 実施記録

| 実施日 | 対象（範囲） | 結果の要約 | 対応 |
|---|---|---|---|
| 2026-11-08 | 最初の push の前の、全履歴を含む点検（点検した時点の7コミット、`main` のみ） | 機械点検: 名義は全件 noreply。メール・認証情報・ユーザー名とドライブのパス・容量の値・追跡してはいけないもの・過去にだけあったファイルは、今の版と全履歴で0件。製品固有の語は、README の断り書きの1か所だけ（履歴の1件も同じ行の追加）、コミットのメッセージは0件。文書間のリンク切れ0件。外部リンクは `github.com` の3件が200、`creativecommons.org` の1件が403（ボット対策）。目視点検: QA 表の1か所に、参考資料の性質を推測させる補足の語があった。台車の仮の値は、参考資料の値と重ならない。画像・第三者の著作物は無し | QA 表の補足の語を一般化した（履歴は残す。ユーザー判断）。予備の実験の記録の版の数字は残す（ユーザー判断）。`creativecommons.org` の403はそのままにした（Claude 判断） |
