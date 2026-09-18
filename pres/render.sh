#!/bin/sh
# render.sh deck.pptx prefix — PDF + JPEG-кадры через LibreOffice в Docker
set -e
deck="$1"; prefix="${2:-slide}"
MSYS_NO_PATHCONV=1 docker run --rm -v "$(pwd -W 2>/dev/null || pwd):/work" pptx-render sh -c \
  "soffice --headless --convert-to pdf '$deck' >/dev/null 2>&1 && rm -f ${prefix}-*.jpg && pdftoppm -jpeg -r ${DPI:-50} '${deck%.pptx}.pdf' '$prefix'"
ls ${prefix}-*.jpg | wc -l
