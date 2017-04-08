#!/usr/bin/env python3
"""Fetches contents of a page from the internet

Usage:
    import story
    story.main(<url>)
"""
from urllib.request import urlopen

print(__name__)

def fetch(url):
    lines = []
    with urlopen(url) as page:
        for line in page:
            lines.append(line.decode('utf-8'))
    return lines


def print_items(items):
    for item in items:
        print(item)


def main(url):
    lines = fetch(url)
    print_items(lines)


if __name__ == '__main__':
    main('http://www.hecr.tifr.res.in/~bsn/GOOD/stories.txt')
