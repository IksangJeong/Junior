"""
데이터베이스 연습 #343
SQLite CRUD 실습
"""
import sqlite3

def setup_343():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_343 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_343_0', 708.93, 'B'), ('item_343_1', 919.5, 'C'), ('item_343_2', 93.9, 'A'), ('item_343_3', 320.05, 'A'), ('item_343_4', 312.33, 'A'), ('item_343_5', 379.57, 'A'), ('item_343_6', 515.72, 'B'), ('item_343_7', 600.46, 'C')]
    ]
    cursor.executemany(
        'INSERT INTO items_343 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_343()
    cursor = conn.cursor()

    print(f"DB 연습 #343")
    cursor.execute('SELECT * FROM items_343')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_343 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
