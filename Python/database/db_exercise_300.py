"""
데이터베이스 연습 #300
SQLite CRUD 실습
"""
import sqlite3

def setup_300():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_300 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_300_0', 424.59, 'B'), ('item_300_1', 693.6, 'B'), ('item_300_2', 506.8, 'C'), ('item_300_3', 921.32, 'C'), ('item_300_4', 730.13, 'C')]
    ]
    cursor.executemany(
        'INSERT INTO items_300 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_300()
    cursor = conn.cursor()

    print(f"DB 연습 #300")
    cursor.execute('SELECT * FROM items_300')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_300 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
