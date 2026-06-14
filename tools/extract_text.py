import fitz
import os

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_CASE_STUDY_DIR = os.path.join(_SCRIPT_DIR, "..", "case-study")

# Get PDFs in case-study folder using os.walk
files = []
for root, dirs, filenames in os.walk(_CASE_STUDY_DIR):
    for filename in filenames:
        if os.path.splitext(filename)[1].lower() == ".pdf":
            files.append(os.path.join(root, filename))

def extract_text(pdf_path) -> str:
    doc = fitz.open(pdf_path)
    pages = [str(page.get_text()) for page in doc]
    doc.close()
    return "\n".join(pages)

# Main
for file in files:
    out_path = os.path.splitext(file)[0] + ".txt"
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(extract_text(file))