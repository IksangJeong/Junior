"""
데이터베이스 연습 #304
SQLite CRUD 실습
"""
import sqlite3

def setup_304():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_304 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_304_0', 109.2, 'B'), ('item_304_1', 720.18, 'A'), ('item_304_2', 684.38, 'A'), ('item_304_3', 15.13, 'C'), ('item_304_4', 978.13, 'A'), ('item_304_5', 412.84, 'B'), ('item_304_6', 482.67, 'A'), ('item_304_7', 758.35, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_304 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_304()
    cursor = conn.cursor()

    print(f"DB 연습 #304")
    cursor.execute('SELECT * FROM items_304')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_304 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
