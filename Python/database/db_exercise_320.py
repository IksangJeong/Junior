"""
데이터베이스 연습 #320
SQLite CRUD 실습
"""
import sqlite3

def setup_320():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_320 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_320_0', 962.46, 'A'), ('item_320_1', 749.41, 'B'), ('item_320_2', 915.3, 'C'), ('item_320_3', 608.36, 'A'), ('item_320_4', 402.67, 'A')]
    ]
    cursor.executemany(
        'INSERT INTO items_320 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_320()
    cursor = conn.cursor()

    print(f"DB 연습 #320")
    cursor.execute('SELECT * FROM items_320')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_320 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
