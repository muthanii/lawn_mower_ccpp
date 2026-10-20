# Lets `latexmk -pdf paper/main.tex` run from the repository root:
# latexmk changes into paper/ so \input and \bibliography paths resolve.
$do_cd = 1;
