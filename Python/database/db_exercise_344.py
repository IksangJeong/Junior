"""
데이터베이스 연습 #344
SQLite CRUD 실습
"""
import sqlite3

def setup_344():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_344 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_344_0', 696.5, 'C'), ('item_344_1', 931.39, 'A'), ('item_344_2', 559.36, 'B'), ('item_344_3', 295.95, 'A'), ('item_344_4', 321.43, 'B'), ('item_344_5', 623.22, 'B'), ('item_344_6', 952.32, 'B'), ('item_344_7', 145.17, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_344 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_344()
    cursor = conn.cursor()

    print(f"DB 연습 #344")
    cursor.execute('SELECT * FROM items_344')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_344 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
