#!/usr/bin/env python3
"""
動画を圧縮するスクリプト（ffmpeg 使用）

使い方:
  python compress_video.py input.mp4
  python compress_video.py input.mp4 -o output.mp4
  python compress_video.py input.mp4 --crf 28 --scale 1280
"""

import argparse
import subprocess
import sys
from pathlib import Path


def get_default_output(path: Path, suffix: str = "_compressed") -> Path:
    """入力パスからデフォルトの出力パスを生成する"""
    stem = path.stem
    if stem.endswith(suffix):
        stem = stem[: -len(suffix)]
    return path.parent / f"{stem}{suffix}{path.suffix}"


def compress_video(
    input_path: Path,
    output_path: Path | None = None,
    *,
    crf: int = 23,
    scale_width: int | None = None,
    preset: str = "medium",
    overwrite: bool = False,
) -> bool:
    """
    動画を H.264 で再エンコードして圧縮する。

    Args:
        input_path: 入力動画のパス
        output_path: 出力パス（None の場合は *_compressed.* に出力）
        crf: 品質 (0–51, 小さいほど高品質・大容量。23 が一般的)
        scale_width: 幅ピクセル（指定時はアスペクト比維持でリサイズ）
        preset: エンコード速度 (ultrafast / fast / medium / slow / veryslow)
        overwrite: 既存ファイルを上書きするか

    Returns:
        成功したら True、失敗したら False
    """
    if not input_path.exists():
        print(f"エラー: 入力ファイルが見つかりません: {input_path}", file=sys.stderr)
        return False

    out = output_path or get_default_output(input_path)
    if out.exists() and not overwrite:
        print(f"エラー: 出力先が既に存在します: {out}", file=sys.stderr)
        print("上書きする場合は --overwrite を指定してください。", file=sys.stderr)
        return False

    out.parent.mkdir(parents=True, exist_ok=True)

    cmd = [
        "ffmpeg",
        "-y" if overwrite else "-n",
        "-i",
        str(input_path),
        "-c:v",
        "libx264",
        "-crf",
        str(crf),
        "-preset",
        preset,
        "-c:a",
        "aac",
        "-b:a",
        "128k",
    ]

    if scale_width is not None and scale_width > 0:
        cmd.extend(["-vf", f"scale={scale_width}:-2"])  # -2 で偶数に

    cmd.append(str(out))

    try:
        subprocess.run(cmd, check=True)
        print(f"完了: {out}")
        return True
    except FileNotFoundError:
        print("エラー: ffmpeg が見つかりません。インストールしてください。", file=sys.stderr)
        return False
    except subprocess.CalledProcessError as e:
        print(f"エラー: ffmpeg が終了コード {e.returncode} で終了しました。", file=sys.stderr)
        return False


def main() -> None:
    parser = argparse.ArgumentParser(
        description="動画を H.264 で圧縮する（ffmpeg 使用）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("input", type=Path, help="入力動画ファイル")
    parser.add_argument("-o", "--output", type=Path, default=None, help="出力ファイル（省略時は *_compressed.*）")
    parser.add_argument(
        "--crf",
        type=int,
        default=23,
        metavar="N",
        help="品質 0–51（小さいほど高品質）。デフォルト: 23",
    )
    parser.add_argument(
        "--scale",
        type=int,
        default=None,
        metavar="WIDTH",
        help="幅ピクセルでリサイズ（アスペクト比維持）",
    )
    parser.add_argument(
        "--preset",
        choices=["ultrafast", "superfast", "veryfast", "faster", "fast", "medium", "slow", "slower", "veryslow"],
        default="medium",
        help="エンコード速度。デフォルト: medium",
    )
    parser.add_argument("--overwrite", action="store_true", help="既存の出力ファイルを上書きする")

    args = parser.parse_args()

    ok = compress_video(
        args.input,
        args.output,
        crf=args.crf,
        scale_width=args.scale,
        preset=args.preset,
        overwrite=args.overwrite,
    )
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
