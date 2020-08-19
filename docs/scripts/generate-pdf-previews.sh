
# generating preview jpg images from experiments pdfs

convert tables/experiments-tutorial.pdf -thumbnail 500x500 -background white +smush 20 -bordercolor white -border 10 tutorial-preview.jpg

convert tables/experiments-cvpr.pdf tables/experiments-cvpr-supplementary.pdf -thumbnail 500x500 -background white +smush 20 -bordercolor white -border 10 cvpr-preview.jpg
