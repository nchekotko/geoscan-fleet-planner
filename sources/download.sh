#!/bin/sh
# Первоисточники ТТХ (ссылки из ТЗ, раздел 6, и руководство Геоскан 401)
cd "$(dirname "$0")"
curl -fL -o Geoscan_201_Manual.pdf  "https://download.geoscan.ru/site-files/201/Geoscan_201_Manual.pdf"
curl -fL -o Geoscan_801.pdf         "https://download.geoscan.ru/site-files/801/Geoscan_801.pdf?v2"
curl -fL -o Gemini_Specs_EN_RUS.pdf "https://download.geoscan.ru/site-files/gemini/Gemini_Specs_EN_RUS.pdf?v2"
curl -fL -o Geoscan_401_Manual.pdf  "https://download.geoscan.ru/site-files/401/Geoscan_401_Manual.pdf"
curl -fL -o MissionPlanner_overview.pdf "https://agtsys.ru/storage/instructions/December2019/AQ2LRTfTZvQcyhtEMPPh.pdf"
