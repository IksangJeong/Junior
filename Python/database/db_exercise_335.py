"""
데이터베이스 연습 #335
SQLite CRUD 실습
"""
import sqlite3

def setup_335():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_335 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_335_0', 122.75, 'C'), ('item_335_1', 207.8, 'C'), ('item_335_2', 188.13, 'C'), ('item_335_3', 36.65, 'C'), ('item_335_4', 467.01, 'C')]
    ]
    cursor.executemany(
        'INSERT INTO items_335 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_335()
    cursor = conn.cursor()

    print(f"DB 연습 #335")
    cursor.execute('SELECT * FROM items_335')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_335 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
