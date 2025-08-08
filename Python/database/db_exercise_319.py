"""
데이터베이스 연습 #319
SQLite CRUD 실습
"""
import sqlite3

def setup_319():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_319 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_319_0', 831.11, 'A'), ('item_319_1', 275.78, 'C'), ('item_319_2', 608.92, 'A'), ('item_319_3', 874.88, 'A'), ('item_319_4', 806.6, 'C'), ('item_319_5', 727.06, 'C'), ('item_319_6', 224.85, 'A'), ('item_319_7', 870.78, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_319 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_319()
    cursor = conn.cursor()

    print(f"DB 연습 #319")
    cursor.execute('SELECT * FROM items_319')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_319 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
