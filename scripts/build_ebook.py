#!/usr/bin/env python3
"""
ROS 2 for Linux Developers - E-Book Generator (PDF & EPUB)
=========================================================
Builds high-quality PDF and EPUB versions of the handbook using Pandoc,
custom typography, LaTeX styling, and modern responsive CSS.

Usage:
    python3 scripts/build_ebook.py [--format {all,pdf,epub}] [--output-dir DIR]
"""

import argparse
import os
import platform
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

# Paths
ROOT_DIR = Path(__file__).resolve().parent.parent
DOCS_DIR = ROOT_DIR / "docs"
EBOOK_DIR = ROOT_DIR / "scripts" / "ebook"
DEFAULT_OUTPUT_DIR = ROOT_DIR / "build" / "ebook"

# E-Book Metadata
BOOK_TITLE = "《Linux 工程师的 ROS 2 工业级实战手册》"
BOOK_SUBTITLE = "The ROS 2 Linux Handbook"
BOOK_AUTHOR = "Gemini 3.8 flash"
BOOK_LANG = "zh"
BOOK_RIGHTS = "Apache-2.0"

# Terminal Colors
class Colors:
    CYAN = "\033[36m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    RED = "\033[31m"
    BOLD = "\033[1m"
    RESET = "\033[0m"


def log_info(msg: str):
    print(f"{Colors.CYAN}>>> {msg}{Colors.RESET}")


def log_success(msg: str):
    print(f"{Colors.GREEN}✓ {msg}{Colors.RESET}")


def log_warning(msg: str):
    print(f"{Colors.YELLOW}⚠ {msg}{Colors.RESET}")


def log_error(msg: str):
    print(f"{Colors.RED}✗ {msg}{Colors.RESET}", file=sys.stderr)


def get_installed_fonts():
    """Query fontconfig for available fonts on the system."""
    if not shutil.which("fc-list"):
        return set()
    try:
        proc = subprocess.run(
            ["fc-list", ":", "family"],
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
        if proc.returncode != 0:
            return set()
        fonts = set()
        for line in proc.stdout.splitlines():
            for f in line.split(","):
                name = f.strip()
                if name:
                    fonts.add(name)
        return fonts
    except Exception:
        return set()


def pick_best_font(candidates, available_fonts):
    """Pick the first matching font candidate that is available."""
    if not available_fonts:
        return candidates[0]
    for c in candidates:
        if c in available_fonts:
            return c
    return candidates[0]


def resolve_fonts():
    """Resolve optimal fonts for current OS."""
    os_name = platform.system()
    available = get_installed_fonts()

    if os_name == "Darwin":  # macOS
        cjk_candidates = ["Songti SC", "STSong", "STHeiti", "Hiragino Sans GB", "PingFang SC"]
        main_candidates = ["Arial Unicode MS", "Helvetica", "Songti SC"]
        mono_candidates = ["Menlo", "Monaco", "SF Mono", "Courier"]
        cjk_mono_candidates = ["Hiragino Sans GB", "STHeiti", "PingFang SC"]
    else:  # Linux / BSD / Docker
        cjk_candidates = [
            "Noto Serif CJK SC",
            "Noto Sans CJK SC",
            "Source Han Serif SC",
            "Source Han Sans SC",
            "WenQuanYi Micro Hei",
            "WenQuanYi Zen Hei",
        ]
        main_candidates = ["Noto Sans", "DejaVu Sans", "Liberation Sans"]
        mono_candidates = ["DejaVu Sans Mono", "Liberation Mono", "Noto Sans Mono"]
        cjk_mono_candidates = ["Noto Sans Mono CJK SC", "WenQuanYi Micro Hei Mono"]

    return {
        "cjk_main": pick_best_font(cjk_candidates, available),
        "main": pick_best_font(main_candidates, available),
        "mono": pick_best_font(mono_candidates, available),
        "cjk_mono": pick_best_font(cjk_mono_candidates, available),
    }


def get_sorted_docs():
    """Retrieve all markdown chapter documents sorted in natural numeric order."""
    doc_files = sorted(DOCS_DIR.glob("*.md"))
    if not doc_files:
        raise RuntimeError(f"No markdown documents found in {DOCS_DIR}")
    return doc_files


def format_size(bytes_num: int) -> str:
    """Format bytes into human-readable string."""
    for unit in ["B", "KB", "MB", "GB"]:
        if bytes_num < 1024:
            return f"{bytes_num:.1f} {unit}"
        bytes_num /= 1024
    return f"{bytes_num:.1f} TB"


def count_pdf_pages(pdf_path: Path) -> int:
    """Estimate page count from PDF binary stream."""
    try:
        import zlib
        data = pdf_path.read_bytes()
        streams = re.findall(rb"stream[\r\n]+(.*?)[\r\n]+endstream", data, re.DOTALL)
        decompressed = []
        for s in streams:
            try:
                decompressed.append(zlib.decompress(s))
            except Exception:
                pass
        all_text = b"\n".join(decompressed) + data
        counts = [int(m) for m in re.findall(rb"/Count\s+(\d+)", all_text)]
        return max(counts) if counts else 0
    except Exception:
        return 0


def build_epub(output_dir: Path, doc_files: list, verbose: bool = False):
    """Build high-quality EPUB 3 e-book."""
    log_info("正在生成 EPUB 电子书...")
    t0 = time.time()
    output_file = output_dir / "ROS2_Linux_Handbook.epub"

    css_file = EBOOK_DIR / "epub.css"
    cover_file = EBOOK_DIR / "cover.png"
    if not cover_file.exists():
        cover_file = EBOOK_DIR / "cover.svg"

    cmd = [
        "pandoc",
        "-t", "epub3",
        "--data-dir", str(EBOOK_DIR),
        "--metadata", f"title={BOOK_TITLE}",
        "--metadata", f"subtitle={BOOK_SUBTITLE}",
        "--metadata", f"author={BOOK_AUTHOR}",
        "--metadata", f"lang={BOOK_LANG}",
        "--metadata", f"rights={BOOK_RIGHTS}",
        "--toc",
        "--toc-depth=3",
    ]

    if css_file.exists():
        cmd.extend(["--css", str(css_file)])

    if cover_file.exists():
        cmd.extend(["--epub-cover-image", str(cover_file)])

    cmd.extend([str(f) for f in doc_files])
    cmd.extend(["-o", str(output_file)])

    if verbose:
        print("Command:", " ".join(cmd))

    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        log_error(f"EPUB 生成失败: {proc.stderr}")
        return False

    elapsed = time.time() - t0
    size_str = format_size(output_file.stat().st_size)
    log_success(f"EPUB 生成成功: {output_file} ({size_str}, 耗时 {elapsed:.2f}s)")
    return True


def build_pdf(output_dir: Path, doc_files: list, engine: str = "xelatex", verbose: bool = False):
    """Build high-quality PDF e-book using LaTeX / Pandoc."""
    log_info(f"正在生成 PDF 电子书 (引擎: {engine})...")
    t0 = time.time()
    output_file = output_dir / "ROS2_Linux_Handbook.pdf"

    if not shutil.which(engine):
        log_error(f"未找到 PDF 渲染引擎 '{engine}'！")
        log_info("请安装 LaTeX 依赖:")
        log_info("  - macOS: brew install basictex && sudo tlmgr install collection-langchinese")
        log_info("  - Ubuntu/Debian: sudo apt-get install -y texlive-xetex texlive-lang-chinese fonts-noto-cjk")
        return False

    fonts = resolve_fonts()
    header_file = EBOOK_DIR / "latex_header.tex"
    lua_filter = EBOOK_DIR / "latex_clean.lua"

    cmd = [
        "pandoc",
        f"--pdf-engine={engine}",
        "-V", "documentclass=report",
        "-V", "colorlinks=true",
        "-V", "linkcolor=blue",
        "-V", "urlcolor=blue",
        "-V", f"CJKmainfont={fonts['cjk_main']}",
        "-V", f"mainfont={fonts['main']}",
        "-V", f"monofont={fonts['mono']}",
        "-V", f"CJKmonofont={fonts['cjk_mono']}",
        "-V", "geometry:top=2.5cm, bottom=2.5cm, left=2.5cm, right=2.5cm",
        "--metadata", f"title={BOOK_TITLE}",
        "--metadata", f"subtitle={BOOK_SUBTITLE}",
        "--metadata", f"author={BOOK_AUTHOR}",
        "--metadata", f"lang={BOOK_LANG}",
        "--toc",
        "--toc-depth=3",
    ]

    if header_file.exists():
        cmd.extend(["-H", str(header_file)])

    if lua_filter.exists():
        cmd.extend(["--lua-filter", str(lua_filter)])

    cmd.extend([str(f) for f in doc_files])
    cmd.extend(["-o", str(output_file)])

    if verbose:
        print("Command:", " ".join(cmd))

    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        log_error(f"PDF 生成失败:\n{proc.stderr}")
        return False

    elapsed = time.time() - t0
    size_str = format_size(output_file.stat().st_size)
    pages = count_pdf_pages(output_file)
    page_str = f", 约 {pages} 页" if pages > 0 else ""
    log_success(f"PDF 生成成功: {output_file} ({size_str}{page_str}, 耗时 {elapsed:.2f}s)")
    return True


def main():
    parser = argparse.ArgumentParser(description="ROS 2 Handbook E-Book Generator")
    parser.add_argument(
        "--format",
        choices=["all", "pdf", "epub"],
        default="all",
        help="生成的目标电子书格式 (默认: all)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"电子书输出目录 (默认: {DEFAULT_OUTPUT_DIR})",
    )
    parser.add_argument(
        "--pdf-engine",
        default="xelatex",
        help="Pandoc PDF 引擎 (默认: xelatex)",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="打印详细执行指令",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="仅检查环境与工具链依赖",
    )
    args = parser.parse_args()

    print("========================================================================")
    print("📖 ROS 2 for Linux Developers - 电子书生成器")
    print("========================================================================")

    # Check pandoc
    if not shutil.which("pandoc"):
        log_error("未找到 'pandoc' 指令！请先安装 pandoc:")
        log_info("  - macOS: brew install pandoc")
        log_info("  - Ubuntu/Debian: sudo apt-get install -y pandoc")
        return 1

    fonts = resolve_fonts()
    print(f"检测到系统: {platform.system()} ({platform.machine()})")
    print(f"排版中文字体: {fonts['cjk_main']} | 等宽字体: {fonts['mono']}")

    if args.check_only:
        log_success("依赖检查完毕！")
        return 0

    args.output_dir.mkdir(parents=True, exist_ok=True)
    doc_files = get_sorted_docs()
    print(f"找到章节文件: 共 {len(doc_files)} 章")

    success = True
    if args.format in ("epub", "all"):
        if not build_epub(args.output_dir, doc_files, verbose=args.verbose):
            success = False

    if args.format in ("pdf", "all"):
        if not build_pdf(args.output_dir, doc_files, engine=args.pdf_engine, verbose=args.verbose):
            success = False

    print("========================================================================")
    if success:
        log_success(f"全部电子书构建完成！产物保存在: {args.output_dir}")
        return 0
    else:
        log_error("部分或全部电子书构建失败，请检查上方日志。")
        return 1


if __name__ == "__main__":
    sys.exit(main())
