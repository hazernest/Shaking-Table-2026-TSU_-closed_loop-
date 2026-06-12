import csv
import importlib
from pathlib import Path


def to_float(value):
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value).strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def load_first_numeric_column(path, scale=1.0):
    suffix = Path(path).suffix.lower()
    if suffix in {".xlsx", ".xlsm"}:
        return _load_excel(path, scale)
    if suffix == ".csv":
        return _load_csv(path, scale)
    raise ValueError("Unsupported file type. Use .xlsx, .xlsm, or .csv")


def _load_excel(path, scale):
    try:
        openpyxl = importlib.import_module("openpyxl")
    except Exception as exc:
        raise RuntimeError("openpyxl is required. Install dependencies from requirements.txt") from exc

    workbook = openpyxl.load_workbook(path, read_only=True, data_only=True)
    worksheet = workbook.active
    values = []
    try:
        for row in worksheet.iter_rows(min_col=1, max_col=1, values_only=True):
            if not row:
                continue
            number = to_float(row[0])
            if number is not None:
                values.append(number * scale)
    finally:
        workbook.close()
    return values


def _load_csv(path, scale):
    values = []
    with open(path, "r", newline="", encoding="utf-8-sig") as file_obj:
        for row in csv.reader(file_obj):
            if not row:
                continue
            number = to_float(row[0])
            if number is not None:
                values.append(number * scale)
    return values
