from pathlib import Path
import markdown

folder = Path(__file__).resolve().parent
renderer = markdown.Markdown(extensions=["tables", "toc"])
body = renderer.convert((folder / "任务书.md").read_text(encoding="utf-8-sig"))
page = """<!doctype html>
<html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>AS2-W 下一轮训练任务书</title>
<style>
body{margin:0;background:#f3f6fa;color:#162337;font:16px/1.7 system-ui,"Microsoft YaHei",sans-serif}
main{max-width:1160px;margin:auto;background:white;padding:clamp(16px,4vw,56px)}
h1,h2{line-height:1.3} h2{margin-top:2.4em;border-bottom:1px solid #d8e1ec;padding-bottom:.4em}
nav{background:#eef4fb;padding:.6rem 1rem;border-radius:8px}
a{color:#145a9b} table{width:100%;border-collapse:collapse} th,td{border:1px solid #d4dfea;padding:.6em;vertical-align:top} th{background:#eaf2fb;text-align:left}
tbody tr:nth-child(even){background:#f7f9fc} code{background:#eef3fa;padding:.15em;overflow-wrap:anywhere}
.table{overflow-x:auto} .table table{min-width:640px}
@media print{body{background:white}main{padding:0}.table table{min-width:0}tr{break-inside:avoid}}
</style>
<main><nav aria-label="目录">TOC</nav>BODY</main></html>"""
body = body.replace("<table>", '<div class="table"><table>').replace("</table>", "</table></div>")
(folder / "任务书.html").write_text(page.replace("TOC", renderer.toc).replace("BODY", body), encoding="utf-8")
