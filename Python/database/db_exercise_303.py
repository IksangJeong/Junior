"""
데이터베이스 연습 #303
SQLite CRUD 실습
"""
import sqlite3

def setup_303():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_303 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_303_0', 51.65, 'C'), ('item_303_1', 88.24, 'B'), ('item_303_2', 659.41, 'C'), ('item_303_3', 484.29, 'A'), ('item_303_4', 829.67, 'B'), ('item_303_5', 933.69, 'C'), ('item_303_6', 278.91, 'A'), ('item_303_7', 619.52, 'C')]
    ]
    cursor.executemany(
        'INSERT INTO items_303 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_303()
    cursor = conn.cursor()

    print(f"DB 연습 #303")
    cursor.execute('SELECT * FROM items_303')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_303 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
